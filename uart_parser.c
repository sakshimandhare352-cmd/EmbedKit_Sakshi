#include <stdio.h>
#include <stdint.h>

#define UART_SOF                0xAAU
#define UART_MAX_PAYLOAD_LEN    16U

#define UART_DEFAULT_TIMEOUT_MS 50U

#define UART_FRAME_COMPLETE     1
#define UART_FRAME_IN_PROGRESS  0
#define UART_CHECKSUM_ERROR     (-1)
#define UART_TIMEOUT_ERROR      (-2)

typedef enum
{
    UART_STATE_WAITING_FOR_SOF = 0,
    UART_STATE_RECEIVING_CMD,
    UART_STATE_RECEIVING_LEN,
    UART_STATE_RECEIVING_PAYLOAD,
    UART_STATE_RECEIVING_CHECKSUM
} uart_parser_state_t;

typedef struct
{
    uart_parser_state_t state;

    uint8_t cmd;
    uint8_t payload_len;
    uint8_t payload[UART_MAX_PAYLOAD_LEN];
    uint8_t payload_index;

    uint8_t checksum;

    uint32_t last_byte_timestamp_ms;
    uint32_t timeout_ms;
} uart_parser_t;

static void uart_parser_reset(uart_parser_t *parser)
{
    parser->state = UART_STATE_WAITING_FOR_SOF;
    parser->cmd = 0U;
    parser->payload_len = 0U;
    parser->payload_index = 0U;
    parser->checksum = 0U;
}

static void uart_parser_init(uart_parser_t *parser, uint32_t timeout_ms)
{
    uart_parser_reset(parser);
    parser->last_byte_timestamp_ms = 0U;
    parser->timeout_ms = timeout_ms;
}

static void uart_print_frame(const uart_parser_t *parser)
{
    uint8_t index;

    printf("FRAME OK CMD=0x%02X LEN=%u PAYLOAD=[",
           (unsigned int)parser->cmd,
           (unsigned int)parser->payload_len);

    for (index = 0U; index < parser->payload_len; ++index)
    {
        if (index > 0U)
        {
            printf(" ");
        }

        printf("%02X", (unsigned int)parser->payload[index]);
    }

    printf("]\n");
}

static int uart_parser_feed_byte(uart_parser_t *parser,
                                 uint8_t byte,
                                 uint32_t timestamp_ms)
{
    uint32_t elapsed_ms;

    /*
     * Timeout must be checked BEFORE accepting the new byte.
     * A timeout of 0 disables timeout checking.
     */
    if ((parser->state != UART_STATE_WAITING_FOR_SOF) &&
        (parser->timeout_ms != 0U))
    {
        elapsed_ms = timestamp_ms - parser->last_byte_timestamp_ms;

        if (elapsed_ms > parser->timeout_ms)
        {
            uart_parser_reset(parser);
            return UART_TIMEOUT_ERROR;
        }
    }

    switch (parser->state)
    {
        case UART_STATE_WAITING_FOR_SOF:
            if (byte == UART_SOF)
            {
                parser->state = UART_STATE_RECEIVING_CMD;
                parser->last_byte_timestamp_ms = timestamp_ms;
                return UART_FRAME_IN_PROGRESS;
            }

            /* Ignore bytes until the start-of-frame marker is found. */
            return UART_FRAME_IN_PROGRESS;

        case UART_STATE_RECEIVING_CMD:
            parser->cmd = byte;
            parser->checksum = byte;
            parser->state = UART_STATE_RECEIVING_LEN;
            parser->last_byte_timestamp_ms = timestamp_ms;
            return UART_FRAME_IN_PROGRESS;

        case UART_STATE_RECEIVING_LEN:
            if (byte > UART_MAX_PAYLOAD_LEN)
            {
                /*
                 * The protocol defines LEN as 0..16.
                 * A value above 16 cannot form a valid frame.
                 * Reset and wait for the next SOF.
                 */
                uart_parser_reset(parser);
                return UART_FRAME_IN_PROGRESS;
            }

            parser->payload_len = byte;
            parser->payload_index = 0U;
            parser->checksum ^= byte;

            if (parser->payload_len == 0U)
            {
                parser->state = UART_STATE_RECEIVING_CHECKSUM;
            }
            else
            {
                parser->state = UART_STATE_RECEIVING_PAYLOAD;
            }

            parser->last_byte_timestamp_ms = timestamp_ms;
            return UART_FRAME_IN_PROGRESS;

        case UART_STATE_RECEIVING_PAYLOAD:
            parser->payload[parser->payload_index] = byte;
            parser->checksum ^= byte;
            ++parser->payload_index;

            if (parser->payload_index >= parser->payload_len)
            {
                parser->state = UART_STATE_RECEIVING_CHECKSUM;
            }

            parser->last_byte_timestamp_ms = timestamp_ms;
            return UART_FRAME_IN_PROGRESS;

        case UART_STATE_RECEIVING_CHECKSUM:
            parser->last_byte_timestamp_ms = timestamp_ms;

            if (byte == parser->checksum)
            {
                int result = UART_FRAME_COMPLETE;
                uart_print_frame(parser);
                uart_parser_reset(parser);
                return result;
            }

            printf("CHECKSUM ERROR expected=0x%02X received=0x%02X\n",
                   (unsigned int)parser->checksum,
                   (unsigned int)byte);

            uart_parser_reset(parser);
            return UART_CHECKSUM_ERROR;

        default:
            uart_parser_reset(parser);
            return UART_FRAME_IN_PROGRESS;
    }
}

static void uart_feed_stream(uart_parser_t *parser,
                             const uint8_t bytes[],
                             const uint32_t timestamps[],
                             uint8_t count)
{
    uint8_t index;

    for (index = 0U; index < count; ++index)
    {
        int result;
        uint32_t gap_ms = 0U;

        /*
         * Capture the gap before calling feed_byte(), because a timeout
         * resets the parser and therefore clears its frame state.
         */
        if ((parser->state != UART_STATE_WAITING_FOR_SOF) &&
            (parser->timeout_ms != 0U))
        {
            gap_ms = timestamps[index] - parser->last_byte_timestamp_ms;
        }

        result = uart_parser_feed_byte(parser, bytes[index], timestamps[index]);

        if (result == UART_FRAME_COMPLETE)
        {
            printf("t=%3ums byte=0x%02X -> frame complete\n",
                   (unsigned int)timestamps[index],
                   (unsigned int)bytes[index]);
        }
        else if (result == UART_TIMEOUT_ERROR)
        {
            printf("t=%3ums byte=0x%02X -> TIMEOUT (%ums gap > %ums) -- parser reset\n",
                   (unsigned int)timestamps[index],
                   (unsigned int)bytes[index],
                   (unsigned int)gap_ms,
                   (unsigned int)UART_DEFAULT_TIMEOUT_MS);

            /*
             * The timed-out byte was not consumed. Re-feed it so a new
             * frame can start immediately when this byte is SOF.
             */
            result = uart_parser_feed_byte(parser, bytes[index], timestamps[index]);

            if (result == UART_FRAME_IN_PROGRESS)
            {
                printf("t=%3ums byte=0x%02X -> receiving... (re-fed after reset)\n",
                       (unsigned int)timestamps[index],
                       (unsigned int)bytes[index]);
            }
        }
        else if (result == UART_CHECKSUM_ERROR)
        {
            printf("t=%3ums byte=0x%02X -> checksum error -- parser reset\n",
                   (unsigned int)timestamps[index],
                   (unsigned int)bytes[index]);
        }
        else
        {
            printf("t=%3ums byte=0x%02X -> receiving...\n",
                   (unsigned int)timestamps[index],
                   (unsigned int)bytes[index]);
        }
    }
}

static void run_test_1(void)
{
    uart_parser_t parser;
    const uint8_t bytes[] =
    {
        0xAAU, 0x01U, 0x03U, 0x10U, 0x20U, 0x30U, 0x02U
    };
    const uint32_t timestamps[] = { 0U, 5U, 10U, 15U, 20U, 25U, 30U };

    printf("\n=== TEST 1: Clean Valid Frame ===\n");

    uart_parser_init(&parser, UART_DEFAULT_TIMEOUT_MS);
    uart_feed_stream(&parser,
                     bytes,
                     timestamps,
                     (uint8_t)(sizeof(bytes) / sizeof(bytes[0])));
}

static void run_test_2(void)
{
    uart_parser_t parser;
    const uint8_t bytes[] =
    {
        0xAAU, 0x01U, 0x03U, 0x10U,
        0xAAU, 0x05U, 0x01U, 0x7FU, 0x7BU
    };
    const uint32_t timestamps[] =
    {
        0U, 5U, 10U, 15U,
        200U, 205U, 210U, 215U, 220U
    };

    printf("\n=== TEST 2: Timeout Mid-Frame, Then Recovery ===\n");

    uart_parser_init(&parser, UART_DEFAULT_TIMEOUT_MS);
    uart_feed_stream(&parser,
                     bytes,
                     timestamps,
                     (uint8_t)(sizeof(bytes) / sizeof(bytes[0])));
}

static void run_test_3(void)
{
    uart_parser_t parser;
    const uint8_t bytes[] =
    {
        0xAAU, 0x03U, 0x01U, 0x55U, 0x57U,
        0xAAU, 0x04U, 0x02U, 0xAAU, 0xBBU, 0x17U
    };
    const uint32_t timestamps[] =
    {
        0U, 5U, 10U, 15U, 20U,
        25U, 30U, 35U, 40U, 45U, 50U
    };

    printf("\n=== TEST 3: Two Valid Frames Back-to-Back ===\n");

    uart_parser_init(&parser, UART_DEFAULT_TIMEOUT_MS);
    uart_feed_stream(&parser,
                     bytes,
                     timestamps,
                     (uint8_t)(sizeof(bytes) / sizeof(bytes[0])));
}

static void run_test_4(void)
{
    uart_parser_t parser;
    const uint8_t bytes[] =
    {
        0xAAU, 0x01U, 0x03U, 0x10U,
        0xAAU, 0x05U, 0x01U, 0x7FU, 0x7BU
    };
    const uint32_t timestamps[] =
    {
        0U, 5U, 10U, 15U,
        200U, 205U, 210U, 215U, 220U
    };

    printf("\n=== TEST 4: Timeout Disabled ===\n");

    uart_parser_init(&parser, 0U);
    uart_feed_stream(&parser,
                     bytes,
                     timestamps,
                     (uint8_t)(sizeof(bytes) / sizeof(bytes[0])));
}

int main(void)
{
    run_test_1();
    run_test_2();
    run_test_3();
    run_test_4();

    return 0;
}
