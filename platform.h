/**
 * @file      platform.h
 * @brief     Declarations for platform-support routines and definitions.
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @author    Christian Klein C. Ramos (Changes made)
 * @date      07 May 2025
 * @note      Enhanced Doxygen documentation and formatting for clarity. Updated author information.
 */

#if !defined(EEE158_EX05_PLATFORM_H_)
#define EEE158_EX05_PLATFORM_H_

#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <stdint.h>

// C linkage should be maintained
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the platform, including any hardware peripherals.
 *
 * This function should be called once at the beginning of the application
 * to set up the necessary hardware components and software modules
 * required for the platform's operation.
 */
void platform_init(void);

/**
 * @brief Executes one iteration of the platform's event processing loop.
 *
 * This function is intended to be called repeatedly within the main application's
 * infinite loop. It handles periodic tasks, event checking, and other
 * platform-specific background activities.
 *
 * @note This is expected to be called within the main application infinite loop.
 */
void platform_do_loop_one(void);

//////////////////////////////////////////////////////////////////////////////
// Pushbutton Interface
//////////////////////////////////////////////////////////////////////////////

/** @brief Pushbutton event mask indicating the on-board button was pressed. */
#define PLATFORM_PB_ONBOARD_PRESS	0x0001

/** @brief Pushbutton event mask indicating the on-board button was released. */
#define PLATFORM_PB_ONBOARD_RELEASE	0x0002

/** @brief Pushbutton event mask for any event (press or release) on the on-board button.
 *         This combines PLATFORM_PB_ONBOARD_PRESS and PLATFORM_PB_ONBOARD_RELEASE.
 */
#define PLATFORM_PB_ONBOARD_MASK	(PLATFORM_PB_ONBOARD_PRESS | PLATFORM_PB_ONBOARD_RELEASE)

/**
 * @brief Determines which pushbutton events have occurred since this function was last called.
 *
 * This function returns a bitmask of pushbutton events. Each bit corresponds to
 * a specific event defined by @c PLATFORM_PB_* macros. The internal event flags
 * are typically cleared after being read.
 *
 * @return A bitmask of @c PLATFORM_PB_* values denoting which event(s) occurred.
 *         Returns 0 if no new events have occurred.
 */
uint16_t platform_pb_get_event(void);

//////////////////////////////////////////////////////////////////////////////
// General Purpose Output (GPO) Interface
//////////////////////////////////////////////////////////////////////////////

/** @brief GPO flag identifier for the on-board LED. */
#define PLATFORM_GPO_LED_ONBOARD	0x0001

/**
 * @brief Modifies the state of General Purpose Outputs (GPOs), such as LEDs.
 *
 * This function allows turning specified GPOs ON or OFF using bitmasks.
 *
 * @param[in] set A bitmask where each set bit corresponds to a GPO to turn ON.
 *                Set to zero if no GPOs are to be turned ON.
 * @param[in] clr A bitmask where each set bit corresponds to a GPO to turn OFF.
 *                Set to zero if no GPOs are to be turned OFF.
 *
 * @note If the same GPO is specified in both @p set and @p clr parameters,
 *       the CLEAR operation (turning OFF) takes precedence.
 */
void platform_gpo_modify(uint16_t set, uint16_t clr);

//////////////////////////////////////////////////////////////////////////////
// Time Specification and Tick Interface
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Structure representing a time specification with second and nanosecond resolution.
 *
 * This structure is used by platform timing routines to represent time values.
 * It is inspired by @c struct timespec from POSIX systems but is not intended
 * to be directly compatible.
 *
 * @note Routines using this structure expect the @c nr_nsec field to be
 *       normalized, i.e., within the interval [0, 999,999,999].
 */
typedef struct platform_timespec_type {
	/** @brief Number of whole seconds elapsed since a reference epoch. */
	uint32_t	nr_sec;

	/**
	 * @brief Number of additional nanoseconds, fractional part of a second.
	 * @note This value is expected to be in the range [0, 999999999].
	 */
	uint32_t	nr_nsec;
} platform_timespec_t;

/**
 * @brief Macro to initialize a @ref platform_timespec_t structure to zero
 *        (0 seconds, 0 nanoseconds).
 */
#define PLATFORM_TIMESPEC_ZERO {0, 0}

/**
 * @brief Compares two @ref platform_timespec_t instances.
 *
 * @param[in] lhs Pointer to the left-hand side @ref platform_timespec_t structure.
 * @param[in] rhs Pointer to the right-hand side @ref platform_timespec_t structure.
 *
 * @return -1 if @p lhs is earlier than @p rhs.
 * @return  0 if @p lhs is equal to @p rhs.
 * @return +1 if @p lhs is later than @p rhs.
 */
int platform_timespec_compare(const platform_timespec_t *lhs,
	const platform_timespec_t *rhs);

/** @brief Defines the duration of a single system tick in microseconds. */
#define	PLATFORM_TICK_PERIOD_US	5000

/**
 * @brief Retrieves the current system tick count since @ref platform_init() was called.
 *
 * The time is returned in a @ref platform_timespec_t structure, representing
 * seconds and nanoseconds. The resolution of this count is determined by
 * @ref PLATFORM_TICK_PERIOD_US.
 *
 * @param[out] tick Pointer to a @ref platform_timespec_t structure where the current
 *                  tick count will be stored. This parameter must not be NULL.
 */
void platform_tick_count(platform_timespec_t *tick);

/**
 * @brief Retrieves a higher-resolution system tick count, if available.
 *
 * This function provides a more precise time measurement if supported by the
 * hardware. If higher resolution is not available, this function behaves
 * identically to @ref platform_tick_count().
 *
 * @param[out] tick Pointer to a @ref platform_timespec_t structure where the current
 *                  high-resolution tick count will be stored. This parameter must not be NULL.
 *
 * @note If unavailable, this function is equivalent to @c platform_tick_count().
 */
void platform_tick_hrcount(platform_timespec_t *tick);

/**
 * @brief Calculates the difference between two tick counts.
 *
 * This function computes `diff = lhs - rhs` and stores the result in @p diff.
 * It correctly handles potential wrap-around of the tick counter, but is
 * generally designed to handle only a single wrap-around event between
 * @p lhs and @p rhs.
 *
 * @param[out] diff Pointer to a @ref platform_timespec_t structure where the calculated
 *                  difference will be stored. This parameter must not be NULL.
 * @param[in]  lhs  Pointer to the minuend @ref platform_timespec_t structure (later time).
 * @param[in]  rhs  Pointer to the subtrahend @ref platform_timespec_t structure (earlier time).
 */
void platform_tick_delta(
	platform_timespec_t *diff,
	const platform_timespec_t *lhs, const platform_timespec_t *rhs
	);

//////////////////////////////////////////////////////////////////////////////
// USART CDC (Communication Device Class) Interface
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Descriptor for asynchronous reception via USART.
 *
 * This structure holds the necessary information for managing an
 * asynchronous data reception operation, including the buffer for
 * incoming data, its maximum length, and completion status.
 */
typedef struct platform_usart_rx_desc_type
{
	/** @brief Pointer to the buffer where received data will be stored. */
	char *buf;

	/** @brief Maximum number of bytes that can be stored in @c buf. */
	uint16_t max_len;

	/**
	 * @brief Type of reception completion event that has occurred.
	 *        This field is volatile as it can be modified by an interrupt service routine.
	 *        See @c PLATFORM_USART_RX_COMPL_* definitions.
	 */
	volatile uint16_t compl_type;

/** @brief Indicates that no reception-completion event has occurred. */
#define PLATFORM_USART_RX_COMPL_NONE	0x0000

/** @brief Indicates that reception completed because the requested amount of data was received or buffer filled. */
#define PLATFORM_USART_RX_COMPL_DATA	0x0001

/**
 * @brief Indicates that reception completed due to a line break detection.
 * @note This completion event is not implemented in this sample.
 */
#define PLATFORM_USART_RX_COMPL_BREAK	0x0002

	/**
	 * @brief Extra information about the completion event, if applicable.
	 *        This field is volatile as it can be modified by an interrupt service routine.
	 */
	volatile union {
		/**
		 * @brief Number of bytes that were actually received into the buffer.
		 * @note This member is valid only if @c compl_type is @c PLATFORM_USART_RX_COMPL_DATA.
		 */
		uint16_t data_len;
	} compl_info;
} platform_usart_rx_async_desc_t;

/**
 * @brief Descriptor for a USART transmission fragment.
 *
 * This structure defines a single block of data to be transmitted,
 * specifying the buffer containing the data and its length. Multiple such
 * descriptors can be used for scatter-gather I/O if supported by the
 * transmission function.
 */
typedef struct platform_usart_tx_desc_type
{
	/** @brief Pointer to the constant buffer containing the data to transmit. */
	const char *buf;

	/** @brief Number of bytes to transmit from @c buf. */
	uint16_t len;
} platform_usart_tx_bufdesc_t;

/**
 * @brief Enqueues an array of data fragments for asynchronous transmission via USART CDC.
 *
 * This function initiates the transmission of data, which can be composed of one
 * or more fragments (buffers). The transmission occurs in the background,
 * allowing the application to continue processing.
 *
 * @note All fragment-array elements and the source buffer(s) they point to
 *       must remain valid (e.g., in scope and unmodified) for the entire
 *       duration of the transmission.
 *
 * @param[in] desc    Pointer to an array of @ref platform_usart_tx_bufdesc_t
 *                    descriptors. Each descriptor defines a fragment of data to transmit.
 *                    This pointer must not be NULL.
 * @param[in] nr_desc The number of descriptors in the @p desc array. Must be greater than 0.
 *
 * @return @c true if the transmission request was successfully enqueued.
 * @return @c false otherwise (e.g., if another transmission is already in
 *         progress, parameters are invalid, or the transmit queue is full).
 */
bool platform_usart_cdc_tx_async(const platform_usart_tx_bufdesc_t *desc,
				 unsigned int nr_desc);

/**
 * @brief Aborts an ongoing asynchronous USART CDC transmission.
 *
 * If a transmission initiated by @ref platform_usart_cdc_tx_async() is in
 * progress, this function will attempt to stop it. Any data already sent
 * or buffered by the hardware might still be transmitted completely.
 * After calling this, @ref platform_usart_cdc_tx_busy() should eventually return @c false.
 */
void platform_usart_cdc_tx_abort(void);

/**
 * @brief Checks if a USART CDC transmission is currently in progress.
 *
 * @return @c true if a transmission initiated by @ref platform_usart_cdc_tx_async()
 *         is active and has not yet completed or been aborted.
 * @return @c false if the USART CDC transmitter is idle.
 */
bool platform_usart_cdc_tx_busy(void);

/**
 * @brief Enqueues a request for asynchronous data reception via USART CDC.
 *
 * This function sets up the platform to receive data into the buffer
 * specified in the provided descriptor. Reception occurs in the background.
 * The completion status and amount of data received can be checked via the
 * @c compl_type and @c compl_info members of the @p desc structure once
 * @ref platform_usart_cdc_rx_busy() returns @c false or a completion event is signaled.
 *
 * @note Both the descriptor itself and the target buffer it points to must
 *       remain valid (e.g., in scope and unmodified) for the entire duration
 *       reception is on-going or until it is explicitly aborted.
 *
 * @param[in] desc Pointer to a @ref platform_usart_rx_async_desc_t descriptor
 *                 that specifies the buffer and conditions for reception.
 *                 This pointer must not be NULL.
 *
 * @return @c true if the reception request was successfully enqueued.
 * @return @c false otherwise (e.g., if another reception is already in
 *         progress, the descriptor is invalid, or the system cannot handle
 *         a new request).
 */
bool platform_usart_cdc_rx_async(platform_usart_rx_async_desc_t *desc);

/**
 * @brief Aborts an ongoing asynchronous USART CDC reception.
 *
 * If a reception initiated by @ref platform_usart_cdc_rx_async() is in
 * progress, this function will attempt to stop it. The state of the
 * reception descriptor's completion fields (@c compl_type, @c compl_info)
 * should be considered unreliable immediately after calling this function
 * until @ref platform_usart_cdc_rx_busy() returns @c false.
 */
void platform_usart_cdc_rx_abort(void);

/**
 * @brief Checks if a USART CDC reception is currently in progress.
 *
 * @return @c true if a reception initiated by @ref platform_usart_cdc_rx_async()
 *         is active and has not yet completed or been aborted.
 * @return @c false if the USART CDC receiver is idle or the operation
 *         has completed or been aborted.
 */
bool platform_usart_cdc_rx_busy(void);

//////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif	// __cplusplus
#endif	// !defined(EEE158_EX05_PLATFORM_H_)