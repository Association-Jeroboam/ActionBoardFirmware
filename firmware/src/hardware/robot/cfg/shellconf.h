#ifndef SHELL_CONF_H
#define SHELL_CONF_H

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Shell maximum input line length.
 */
#if !defined(SHELL_MAX_LINE_LENGTH) || defined(__DOXYGEN__)
#define SHELL_MAX_LINE_LENGTH       64
#endif

/**
 * @brief   Shell maximum arguments per command.
 */
#if !defined(SHELL_MAX_ARGUMENTS) || defined(__DOXYGEN__)
#define SHELL_MAX_ARGUMENTS         4
#endif

/**
 * @brief   Shell maximum command history.
 */
#if !defined(SHELL_MAX_HIST_BUFF) || defined(__DOXYGEN__)
#define SHELL_MAX_HIST_BUFF         8 * SHELL_MAX_LINE_LENGTH
#endif

/**
 * @brief   Enable shell command history
 */
#if !defined(SHELL_USE_HISTORY) || defined(__DOXYGEN__)
#define SHELL_USE_HISTORY           TRUE
#endif

/**
 * @brief   Enable shell command completion
 */
#if !defined(SHELL_USE_COMPLETION) || defined(__DOXYGEN__)
#define SHELL_USE_COMPLETION        TRUE
#endif

/**
 * @brief   Shell Maximum Completions (Set to max commands with common prefix)
 */
#if !defined(SHELL_MAX_COMPLETIONS) || defined(__DOXYGEN__)
#define SHELL_MAX_COMPLETIONS       8
#endif

/**
 * @brief   Enable shell escape sequence processing
 */
#if !defined(SHELL_USE_ESC_SEQ) || defined(__DOXYGEN__)
#define SHELL_USE_ESC_SEQ           TRUE
#endif

/**
 * @brief   Prompt string
 */
#if !defined(SHELL_PROMPT_STR) || defined(__DOXYGEN__)
#ifdef RED_ROBOT
#define SHELL_PROMPT_STR            "RedRobotActionBoard> "
#elif defined BLUE_ROBOT
#define SHELL_PROMPT_STR            "BlueRobotActionBoard> "
#endif /* RED_ROBOT || BLUE_ROBOT */
#endif

/**
 * @brief   Newline string
 */
#if !defined(SHELL_NEWLINE_STR) || defined(__DOXYGEN__)
#define SHELL_NEWLINE_STR            "\r\n"
#endif

#if !defined(SHELL_CMD_EXIT_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_EXIT_ENABLED              FALSE
#endif

#if !defined(SHELL_CMD_INFO_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_INFO_ENABLED              TRUE
#endif

#if !defined(SHELL_CMD_ECHO_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_ECHO_ENABLED              TRUE
#endif

#if !defined(SHELL_CMD_SYSTIME_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_SYSTIME_ENABLED           TRUE
#endif

#if !defined(SHELL_CMD_MEM_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_MEM_ENABLED               TRUE
#endif

#if !defined(SHELL_CMD_THREADS_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_THREADS_ENABLED           TRUE
#endif

#if !defined(SHELL_CMD_TEST_ENABLED) || defined(__DOXYGEN__)
#define SHELL_CMD_TEST_ENABLED              FALSE
#endif

#if !defined(SHELL_CMD_TEST_WA_SIZE) || defined(__DOXYGEN__)
#define SHELL_CMD_TEST_WA_SIZE              THD_WORKING_AREA_SIZE(256)
#endif


#endif
