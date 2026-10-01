#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/twai.h"
#include "esp_timer.h"
#include <math.h>

extern "C" {
#include <canard.h>
}

// ============================================================
// PINS
// ============================================================

#define SDA_PIN 8
#define SCL_PIN 9

#define CAN_TX_PIN 43
#define CAN_RX_PIN 44

// ============================================================
// DRONECAN NODE IDs
// ============================================================

#define ESP32_NODE_ID 120
#define RM3100_NODE_ID 10

// ============================================================
// DRONECAN DATA TYPES
// ============================================================

#define DNA_DATA_TYPE_ID 1
#define DNA_SIGNATURE 0x0B2A812620A11D40ULL

#define MAG_DATA_TYPE_ID 1001
#define MAG_SIGNATURE 0xE2A7D4A9460BC2F2ULL

// ============================================================
// MAGNETOMETER CALIBRATION
// ============================================================

const float OFFSET_X = 0.002686f;
const float OFFSET_Y = 0.010132f;
const float OFFSET_Z = 0.010010f;

const float SCALE_X = 1.002611f;
const float SCALE_Y = 1.043482f;
const float SCALE_Z = 0.957602f;

// ============================================================
// FINAL HEADING OFFSET
// ============================================================

// North test showed 27.6 deg.
// Previous offset was +3.6 deg.
// Final correction = 3.6 - 27.6 = -24.0 deg.

const float HEADING_OFFSET = -24.0f;

// ============================================================
// OLED
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE
);

// ============================================================
// LIBCANARD
// ============================================================

static CanardInstance canard;
static uint8_t canardMemoryPool[4096];

static uint8_t dnaTxTransferID = 0;

// ============================================================
// STATE
// ============================================================

static uint8_t rmUID[16];
static uint8_t rmUIDLength = 0;

static bool rmAllocated = false;

static uint32_t lastDNARequestMs = 0;
static uint32_t lastMagMs = 0;
static uint32_t magCounter = 0;

static float rawX = 0.0f;
static float rawY = 0.0f;
static float rawZ = 0.0f;

static float magX = 0.0f;
static float magY = 0.0f;
static float magZ = 0.0f;

static float magTotal = 0.0f;
static float rawHeadingDeg = 0.0f;
static float headingDeg = 0.0f;

// ============================================================
// ANGLE
// ============================================================

float normalizeHeading(float angle)
{
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    return angle;
}

float calculateHeading(float x, float y)
{
    float heading =
        atan2f(-y, x) *
        180.0f /
        PI;

    heading =
        normalizeHeading(heading);

    heading +=
        HEADING_OFFSET;

    return normalizeHeading(
        heading
    );
}

// ============================================================
// OLED
// ============================================================

void showOLED()
{
    u8g2.clearBuffer();

    u8g2.setFont(
        u8g2_font_6x12_tr
    );

    char buf[32];

    if (!rmAllocated)
    {
        u8g2.drawStr(
            0,
            12,
            "BDM / RM3100"
        );

        u8g2.drawStr(
            0,
            27,
            "DroneCAN allocator"
        );

        snprintf(
            buf,
            sizeof(buf),
            "UID: %u/16",
            rmUIDLength
        );

        u8g2.drawStr(
            0,
            42,
            buf
        );

        u8g2.drawStr(
            0,
            57,
            "Allocating..."
        );

        u8g2.sendBuffer();

        return;
    }

    snprintf(
        buf,
        sizeof(buf),
        "HEAD: %6.1f deg",
        headingDeg
    );

    u8g2.drawStr(
        0,
        11,
        buf
    );

    snprintf(
        buf,
        sizeof(buf),
        "X: %+0.3f G",
        magX
    );

    u8g2.drawStr(
        0,
        24,
        buf
    );

    snprintf(
        buf,
        sizeof(buf),
        "Y: %+0.3f G",
        magY
    );

    u8g2.drawStr(
        0,
        37,
        buf
    );

    snprintf(
        buf,
        sizeof(buf),
        "Z: %+0.3f G",
        magZ
    );

    u8g2.drawStr(
        0,
        50,
        buf
    );

    snprintf(
        buf,
        sizeof(buf),
        "|B|: %.3f G",
        magTotal
    );

    u8g2.drawStr(
        0,
        63,
        buf
    );

    u8g2.sendBuffer();
}

// ============================================================
// CANARD TX -> TWAI
// ============================================================

void processCanardTX()
{
    while (true)
    {
        const CanardCANFrame* txf =
            canardPeekTxQueue(
                &canard
            );

        if (txf == nullptr)
        {
            break;
        }

        twai_message_t tx = {};

        tx.identifier =
            txf->id &
            CANARD_CAN_EXT_ID_MASK;

        tx.extd = 1;
        tx.rtr = 0;

        tx.data_length_code =
            txf->data_len;

        for (
            uint8_t i = 0;
            i < txf->data_len;
            i++
        )
        {
            tx.data[i] =
                txf->data[i];
        }

        esp_err_t result =
            twai_transmit(
                &tx,
                pdMS_TO_TICKS(50)
            );

        if (result == ESP_OK)
        {
            canardPopTxQueue(
                &canard
            );
        }
        else
        {
            break;
        }
    }
}

// ============================================================
// DYNAMIC NODE ID
// ============================================================

void sendDNAResponse()
{
    if (rmUIDLength == 0)
    {
        return;
    }

    uint8_t payload[17] = {};

    payload[0] =
        (uint8_t)(
            RM3100_NODE_ID
            << 1
        );

    memcpy(
        &payload[1],
        rmUID,
        rmUIDLength
    );

    canardBroadcast(
        &canard,
        DNA_SIGNATURE,
        DNA_DATA_TYPE_ID,
        &dnaTxTransferID,
        CANARD_TRANSFER_PRIORITY_LOW,
        payload,
        rmUIDLength + 1
    );

    processCanardTX();
}

void handleDNARequest(
    CanardRxTransfer* transfer
)
{
    if (
        transfer->source_node_id != 0 ||
        transfer->payload_len < 2
    )
    {
        return;
    }

    uint8_t header = 0;

    canardDecodeScalar(
        transfer,
        0,
        8,
        false,
        &header
    );

    bool firstPart =
        (header & 0x01) != 0;

    if (
        rmUIDLength > 0 &&
        millis() -
        lastDNARequestMs >
        500
    )
    {
        rmUIDLength = 0;
    }

    lastDNARequestMs =
        millis();

    if (firstPart)
    {
        rmUIDLength = 0;
        rmAllocated = false;
    }

    uint8_t chunkLength =
        transfer->payload_len - 1;

    if (chunkLength > 6)
    {
        chunkLength = 6;
    }

    for (
        uint8_t i = 0;
        i < chunkLength;
        i++
    )
    {
        uint8_t value = 0;

        canardDecodeScalar(
            transfer,
            8 + i * 8,
            8,
            false,
            &value
        );

        if (
            rmUIDLength <
            16
        )
        {
            rmUID[
                rmUIDLength++
            ] = value;
        }
    }

    sendDNAResponse();

    if (
        rmUIDLength >=
        16
    )
    {
        rmAllocated =
            true;

        Serial.println(
            "RM3100 Node ID = 10"
        );
    }

    showOLED();
}

// ============================================================
// MAGNETOMETER
// ============================================================

void handleMagneticField(
    CanardRxTransfer* transfer
)
{
    if (
        transfer->source_node_id !=
        RM3100_NODE_ID
    )
    {
        return;
    }

    if (
        transfer->payload_len <
        6
    )
    {
        return;
    }

    uint16_t dataX = 0;
    uint16_t dataY = 0;
    uint16_t dataZ = 0;

    canardDecodeScalar(
        transfer,
        0,
        16,
        false,
        &dataX
    );

    canardDecodeScalar(
        transfer,
        16,
        16,
        false,
        &dataY
    );

    canardDecodeScalar(
        transfer,
        32,
        16,
        false,
        &dataZ
    );

    rawX =
        canardConvertFloat16ToNativeFloat(
            dataX
        );

    rawY =
        canardConvertFloat16ToNativeFloat(
            dataY
        );

    rawZ =
        canardConvertFloat16ToNativeFloat(
            dataZ
        );

    // ========================================================
    // APPLY CALIBRATION
    // ========================================================

    magX =
        (
            rawX -
            OFFSET_X
        ) *
        SCALE_X;

    magY =
        (
            rawY -
            OFFSET_Y
        ) *
        SCALE_Y;

    magZ =
        (
            rawZ -
            OFFSET_Z
        ) *
        SCALE_Z;

    // ========================================================
    // FIELD MAGNITUDE
    // ========================================================

    magTotal =
        sqrtf(
            magX * magX +
            magY * magY +
            magZ * magZ
        );

    // ========================================================
    // HEADING
    // ========================================================

    rawHeadingDeg =
        normalizeHeading(
            atan2f(
                -magY,
                magX
            ) *
            180.0f /
            PI
        );

    headingDeg =
        calculateHeading(
            magX,
            magY
        );

    rmAllocated =
        true;

    lastMagMs =
        millis();

    magCounter++;

    // ========================================================
    // SERIAL
    // ========================================================

    static uint32_t lastPrint = 0;

    if (
        millis() -
        lastPrint >=
        250
    )
    {
        lastPrint =
            millis();

        Serial.print(
            "MAG #"
        );

        Serial.print(
            magCounter
        );

        Serial.print(
            " RAW="
        );

        Serial.print(
            rawHeadingDeg,
            1
        );

        Serial.print(
            " FINAL="
        );

        Serial.print(
            headingDeg,
            1
        );

        Serial.print(
            " deg"
        );

        Serial.print(
            " X="
        );

        Serial.print(
            magX,
            4
        );

        Serial.print(
            " Y="
        );

        Serial.print(
            magY,
            4
        );

        Serial.print(
            " Z="
        );

        Serial.print(
            magZ,
            4
        );

        Serial.print(
            " |B|="
        );

        Serial.println(
            magTotal,
            4
        );
    }

    // ========================================================
    // OLED
    // ========================================================

    static uint32_t
        lastDisplay = 0;

    if (
        millis() -
        lastDisplay >=
        100
    )
    {
        lastDisplay =
            millis();

        showOLED();
    }
}

// ============================================================
// DRONECAN CALLBACK
// ============================================================

static void onTransferReceived(
    CanardInstance* ins,
    CanardRxTransfer* transfer
)
{
    (void)ins;

    if (
        transfer->transfer_type !=
        CanardTransferTypeBroadcast
    )
    {
        return;
    }

    if (
        transfer->data_type_id ==
        DNA_DATA_TYPE_ID
    )
    {
        handleDNARequest(
            transfer
        );
    }
    else if (
        transfer->data_type_id ==
        MAG_DATA_TYPE_ID
    )
    {
        handleMagneticField(
            transfer
        );
    }
}

// ============================================================
// DRONECAN FILTER
// ============================================================

static bool shouldAcceptTransfer(
    const CanardInstance* ins,
    uint64_t* outDataTypeSignature,
    uint16_t dataTypeID,
    CanardTransferType transferType,
    uint8_t sourceNodeID
)
{
    (void)ins;
    (void)sourceNodeID;

    if (
        transferType !=
        CanardTransferTypeBroadcast
    )
    {
        return false;
    }

    if (
        dataTypeID ==
        DNA_DATA_TYPE_ID
    )
    {
        *outDataTypeSignature =
            DNA_SIGNATURE;

        return true;
    }

    if (
        dataTypeID ==
        MAG_DATA_TYPE_ID
    )
    {
        *outDataTypeSignature =
            MAG_SIGNATURE;

        return true;
    }

    return false;
}

// ============================================================
// TWAI -> LIBCANARD
// ============================================================

void processCANFrame(
    const twai_message_t& msg
)
{
    if (!msg.extd)
    {
        return;
    }

    CanardCANFrame frame = {};

    frame.id =
        (
            msg.identifier &
            CANARD_CAN_EXT_ID_MASK
        ) |
        CANARD_CAN_FRAME_EFF;

    frame.data_len =
        msg.data_length_code;

    for (
        uint8_t i = 0;
        i <
        msg.data_length_code;
        i++
    )
    {
        frame.data[i] =
            msg.data[i];
    }

    canardHandleRxFrame(
        &canard,
        &frame,
        (uint64_t)
            esp_timer_get_time()
    );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(
        1200
    );

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        "BDM RM3100 FINAL WORK MODE"
    );

    Serial.println(
        "=============================="
    );

    Serial.print(
        "HEADING OFFSET = "
    );

    Serial.print(
        HEADING_OFFSET,
        1
    );

    Serial.println(
        " deg"
    );

    // OLED

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    u8g2.setI2CAddress(
        0x3C << 1
    );

    u8g2.begin();

    showOLED();

    // libcanard

    canardInit(
        &canard,
        canardMemoryPool,
        sizeof(
            canardMemoryPool
        ),
        onTransferReceived,
        shouldAcceptTransfer,
        nullptr
    );

    canardSetLocalNodeID(
        &canard,
        ESP32_NODE_ID
    );

    // CAN

    twai_general_config_t gConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)
                CAN_TX_PIN,
            (gpio_num_t)
                CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t tConfig =
        TWAI_TIMING_CONFIG_1MBITS();

    twai_filter_config_t fConfig =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result =
        twai_driver_install(
            &gConfig,
            &tConfig,
            &fConfig
        );

    if (
        result !=
        ESP_OK
    )
    {
        Serial.println(
            "TWAI install FAILED"
        );

        return;
    }

    result =
        twai_start();

    if (
        result !=
        ESP_OK
    )
    {
        Serial.println(
            "TWAI start FAILED"
        );

        return;
    }

    Serial.println(
        "CAN started at 1 Mbit/s"
    );

    Serial.println(
        "Waiting for RM3100..."
    );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    twai_message_t message;

    if (
        twai_receive(
            &message,
            pdMS_TO_TICKS(20)
        ) ==
        ESP_OK
    )
    {
        if (
            message.extd
        )
        {
            processCANFrame(
                message
            );
        }
    }

    processCanardTX();

    static uint32_t
        lastCleanup = 0;

    if (
        millis() -
        lastCleanup >=
        1000
    )
    {
        lastCleanup =
            millis();

        canardCleanupStaleTransfers(
            &canard,
            esp_timer_get_time()
        );

        if (
            rmAllocated &&
            lastMagMs != 0 &&
            millis() -
                lastMagMs >
                3000
        )
        {
            Serial.println(
                "WARNING: RM3100 DATA TIMEOUT"
            );
        }
    }
}