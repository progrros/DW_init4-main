#include "main.h"
#include "tim.h"
#include <stdlib.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <deca_types.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include <config_options.h>
#include "string.h"
#include "stdio.h"
#include <time.h>
#include "usart.h"

#if defined(TEST_DS_TWR_INITIATOR_STS)

extern float received_data[6];        // 6 floats = 24 bytes 

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#if INITIATOR_TYPE == 'E'
	#define ANT_DELAY 16525//16525////16405//16549//16550
#elif  INITIATOR_TYPE == 'F'
	#define ANT_DELAY 16525//16525////16405//16549//16550
#elif  INITIATOR_TYPE == 'G'
	#define ANT_DELAY 16525//16525////16405//16549//16550
#endif
#define TX_ANT_DLY ANT_DELAY //16525
#define RX_ANT_DLY ANT_DELAY  //16525

/* Number of Anchors */
#define NUMBER_OF_ANCHORS 4// choose 4 or 6
/* Index of address in msg */
#define SOURCE_ADDRESS_IDX 7
#define DESTINATION_ADDRESS_IDX 5

/* ====> Addresses of anchors and current anchor number <==== */
uint8_t current_anchor = 1;

#if 0
static uint8_t address_of_A[] = {'A', 1};//{'A', 1};
static uint8_t address_of_B[] = {'A', 1};//{'B', 2};
static uint8_t address_of_C[] = {'A', 1};//{'C', 3};
static uint8_t address_of_D[] = {'A', 1};//{'D', 4};
#endif

#if 0
static uint8_t address_of_A[] = {'B', 2};//{'A', 1};
static uint8_t address_of_B[] = {'B', 2};//{'B', 2};
static uint8_t address_of_C[] = {'B', 2};//{'C', 3};
static uint8_t address_of_D[] = {'B', 2};//{'D', 4};
#endif

#if 0
static uint8_t address_of_A[] = {'C', 3};//{'A', 1};
static uint8_t address_of_B[] = {'C', 3};//{'B', 2};
static uint8_t address_of_C[] = {'C', 3};//{'C', 3};
static uint8_t address_of_D[] = {'C', 3};//{'D', 4};
#endif

#if 0
static uint8_t address_of_A[] = {'D', 4};//{'A', 1};
static uint8_t address_of_B[] = {'D', 4};//{'B', 2};
static uint8_t address_of_C[] = {'D', 4};//{'C', 3};
static uint8_t address_of_D[] = {'D', 4};//{'D', 4};
#endif

#if 0
static uint8_t address_of_A[] = {'E', 5};//{'A', 1};
static uint8_t address_of_B[] = {'E', 5};//{'B', 2};
static uint8_t address_of_C[] = {'E', 5};//{'C', 3};
static uint8_t address_of_D[] = {'E', 5};//{'D', 4};
#endif

#if 0
static uint8_t address_of_A[] = {'F', 6};//{'A', 1};
static uint8_t address_of_B[] = {'F', 6};//{'B', 2};
static uint8_t address_of_C[] = {'F', 6};//{'C', 3};
static uint8_t address_of_D[] = {'F', 6};//{'D', 4};
#endif


#if 1
	#if NUMBER_OF_ANCHORS == 4
		static uint8_t address_of_A[] = {'A', 1};//{'A', 1};
		static uint8_t address_of_B[] = {'B', 2};//{'B', 2};
		static uint8_t address_of_C[] = {'C', 3};//{'C', 3};
		static uint8_t address_of_D[] = {'D', 4};//{'D', 4};
//		static uint8_t address_of_A[] = {'A', 1};//{'A', 1};
//		static uint8_t address_of_B[] = {'B', 2};//{'B', 2};
//		static uint8_t address_of_D[] = {'D', 4};//{'D', 4};
//		static uint8_t address_of_C[] = {'E', 5};//{'C', 3};
	#elif NUMBER_OF_ANCHORS == 6
		static uint8_t address_of_A[] = {'A', 1};//{'A', 1};
		static uint8_t address_of_B[] = {'B', 2};//{'B', 2};
		static uint8_t address_of_C[] = {'C', 3};//{'C', 3};
		static uint8_t address_of_D[] = {'D', 4};//{'D', 4};
		static uint8_t address_of_E[] = {'E', 5};//{'E', 5};
		static uint8_t address_of_F[] = {'F', 6};//{'F', 6};
	#endif
#endif

/* Frames used in the ranging process. See NOTE 3 below. */
/* ==== Address of MAIN Initiator is -> 'V', 'E'==== */
/*
 * On index 5 and 6 in TX messages swap Address of current anchor
 * on index 7 and 8 in RX message swap Address of current anchor
 */
#if INITIATOR_TYPE == 'E'
		static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
		static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0};
		static uint8_t tx_final_msg[] = {
            0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE2,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // Extra 24 bytes initialized to zero
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };
#elif  INITIATOR_TYPE == 'F'
		static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'F', 0xE0, 0, 0};
		static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'F', 'W', 'A', 0xE1, 0, 0};
		static uint8_t tx_final_msg[] = {
            0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'F', 0xE2,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // Extra 24 bytes initialized to zero
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };

#elif  INITIATOR_TYPE == 'G'
		static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'G', 0xE0, 0, 0};
		static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'G', 'W', 'A', 0xE1, 0, 0};
		static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'G', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif



/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS (290 + CPU_COMP_RESP)//(290 + 327) //(290 + CPU_COMP) //

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 550 us with above configuration. */
// 912 represents a fixed delay of 912 microseconds (µs). This delay is likely associated with the timing requirements in an 
//Ultra-Wideband (UWB) communication protocol, such as the IEEE 802.15.4z standard.
#define RESP_RX_TO_FINAL_TX_DLY_UUS (480 + 912 + CPU_COMP_INIT)


/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 300

/* Hold the amount of errors that have occurred */
static uint32_t errors[23] = {0};

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

//my var
uint32_t timtick_1;
uint32_t timtick_2;
uint32_t diff;
uint8_t randomDelay;



static dwt_sts_cp_key_t cp_key =
{
        0x14EB220F,0xF86050A8,0xD1D336AA,0x14148674
};

/*
 * 128-bit initial value for the nonce to be programmed into the CP_IV register.
 *
 * The IV, like the key, needs to be known and programmed the same at both units performing the SS-TWR.
 * It can be considered as an extension of the KEY. The low 32 bits of the IV is the counter.
 * In a real application for any particular key the value of the IV including the count should not be reused,
 * i.e. if the counter value wraps the upper 96-bits of the IV should be changed, e.g. incremented.
 *
 * Here we use a default IV as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_iv_t cp_iv =
{
        0x1F9A3DE4,0xD37EC3CA,0xC44FA8FB,0x362EEB34
};

/*
 * The 'poll' message initiating the ranging exchange includes a 32-bit counter which is part
 * of the IV used to generate the scrambled timestamp sequence (STS) in the transmitted packet.
 */
static void send_tx_poll_msg(void)
{
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_initiator_sts()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ds_twr_initiator_sts(void)
{
    int16_t stsQual; /* This will contain STS quality index and status */
    int goodSts = 0; /* Used for checking STS quality in received signal */
    uint8_t firstLoopFlag = 0; /* Used for checking if the program has gone through the main loop for the first time */

    /* Reset DW IC */
    my_reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    if(config_options.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
    }

    /* ====> Enable frame filtering <==== */
#if INITIATOR_TYPE == 'E'
    // Enable 802.15.4 frame filtering to process only relevant data frames
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
    // Set the Personal Area Network (PAN) ID to 0xDECA
	dwt_setpanid(0xDECA);
    // Assign the device's 16-bit short address to 0x4556
	dwt_setaddress16(0x4556);
#elif INITIATOR_TYPE == 'F'
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
   	dwt_setpanid(0xDECA);
   	dwt_setaddress16(0x4656);
#elif  INITIATOR_TYPE == 'G'
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
	dwt_setpanid(0xDECA);
	dwt_setaddress16(0x4756);
#endif

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 14, 17 and 18 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Set expected response's timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay, this value can be set here once for all. */
    set_resp_rx_timeout(RESP_RX_TIMEOUT_UUS, &config_options);

    // for random
    srand(time(NULL));

    /* Loop for user defined number of ranges. */
    while (1)
    {
        /*
         * Set STS encryption key and IV (nonce).
         * See NOTE 16 below.
         */
        if (!firstLoopFlag)
        {
            /*
             * On first loop, configure the STS key & IV, then load them.
             */
            dwt_configurestskey(&cp_key);
            dwt_configurestsiv(&cp_iv);
            dwt_configurestsloadiv();
            firstLoopFlag = 1;
        }
        else
        {
            /*
             * On subsequent loops, we only need to reload the lower 32 bits of STS IV.
             */
            dwt_writetodevice(STS_IV0_ID, 0, 4, (uint8_t *)&cp_iv);
            dwt_configurestsloadiv();
        }

        /* ====> Check current number of anchor and put destination address <==== */
        switch (current_anchor)
        {
			case 1: // anchor 1 A
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_A[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_A[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_A[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_A[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_A[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_A[1];
				//test_run_info((unsigned char *)"Anchor: A");
				break;
			}

			case 2: // anchor 2 B
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_B[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_B[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_B[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_B[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_B[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_B[1];
				//test_run_info((unsigned char *)"Anchor: B");
				break;
			}

			case 3: //anchor 3 C
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_C[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_C[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_C[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_C[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_C[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_C[1];
				//test_run_info((unsigned char *)"Anchor: C");
				break;
			}
#if NUMBER_OF_ANCHORS == 4
			case 0: // anchor 4 D
#elif NUMBER_OF_ANCHORS == 6
			case 4:
#endif
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_D[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_D[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_D[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_D[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_D[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_D[1];
				//test_run_info((unsigned char *)"Anchor: D");
				break;
			}
#if NUMBER_OF_ANCHORS == 6
			case 5:
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_E[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_E[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_E[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_E[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_E[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_E[1];
				//test_run_info((unsigned char *)"Anchor: D");
				break;
			}
			case 0: // anchor 6 F
			{
				tx_poll_msg[DESTINATION_ADDRESS_IDX] = address_of_F[0];
				tx_poll_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_F[1];

				rx_resp_msg[SOURCE_ADDRESS_IDX] = address_of_F[0];
				rx_resp_msg[SOURCE_ADDRESS_IDX + 1] = address_of_F[1];

				tx_final_msg[DESTINATION_ADDRESS_IDX] = address_of_F[0];
				tx_final_msg[DESTINATION_ADDRESS_IDX + 1] = address_of_F[1];
				//test_run_info((unsigned char *)"Anchor: D");
				break;
			}
#endif


		}

        /*
         * Send the poll message to the responder.
         */
        send_tx_poll_msg();

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        /* ====> Take first tic of timer <==== */
        //timtick_1 = __HAL_TIM_GET_COUNTER(&htim2);
        /*
         * Need to check the STS has been received and is good.
         */
        goodSts = dwt_readstsquality(&stsQual);

        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;

        /*
         * Here we are checking for a good frame and good STS quality.
         */
        if ((status_reg & SYS_STATUS_RXFCG_BIT_MASK) && (goodSts >= 0))
        {
            uint32_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check that the frame is the expected response from the companion "DS TWR responder STS" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t final_tx_time;
                    uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                    int ret = DWT_ERROR;

                    /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                    poll_tx_ts = get_tx_timestamp_u64();
                    resp_rx_ts = get_rx_timestamp_u64();

                    /* Compute final message transmission time. See NOTE 19 below. */
                    final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(final_tx_time);

                    final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    /* Write all timestamps in the final message. See NOTE 19 below. */
                    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);


                    /* Write and send final message. See NOTE 7 below. */
                    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                     ///
                     memcpy(&tx_final_msg[22], (uint8_t*)received_data, sizeof(received_data)); 

                    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging bit set. */
                    ret = dwt_starttx(DWT_START_TX_DELAYED);


                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */
                    if (ret == DWT_SUCCESS)
                    {
                       /* Poll DW IC until TX frame sent event set. See NOTE 8 below. */
                       while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                       { };

                       /* Clear TXFRS event. */
                       dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                       /* Increment frame sequence number after transmission of the final message (modulo 256). */
                       frame_seq_nb++;
                       /* Increase a current anchor number */
                       current_anchor = (current_anchor + 1 ) % NUMBER_OF_ANCHORS;
                    }
                }
                else
                {
                    errors[BAD_FRAME_ERR_IDX] += 1;
                    /* Increase a current anchor number */
                    current_anchor = (current_anchor + 1 ) % NUMBER_OF_ANCHORS;
                }
            }
            else
            {
                errors[RTO_ERR_IDX] += 1;
                /* Increase a current anchor number */
                current_anchor = (current_anchor + 1 ) % NUMBER_OF_ANCHORS;
            }
        }
        else
        {
#if 0
            check_for_status_errors(status_reg, errors);

            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
            {
                errors[BAD_FRAME_ERR_IDX] += 1;
            }
            if (goodSts < 0)
            {
                errors[PREAMBLE_COUNT_ERR_IDX] += 1;
            }
            if (stsQual <= 0)
            {
                errors[CP_QUAL_ERR_IDX] += 1;
            }
#endif
            /* Increase a current anchor number */
            current_anchor = (current_anchor + 1 ) % NUMBER_OF_ANCHORS;
        }

        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Execute a delay between ranging exchanges. */
        /* ====> Take second tic of timer to check all time <==== */
		//timtick_2 = __HAL_TIM_GET_COUNTER(&htim2);
		/* ====> Take difference  <==== */
		//diff = timtick_2 - timtick_1;
        randomDelay = rand() % 5;
        Sleep(randomDelay);
    }

}
#endif
