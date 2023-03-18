#ifndef MODULE
#define	MODULE

/* Initialize parameter constants */
static const char *SUCCESS_COMMAND_SIGN[] = { "OK\r\n", "\r\n\r\n" };

static const char *ERROR_COMMAND_SIGN[] = { "ERROR\r\n", "ERROR" };

static const char *PASSIVE_RESPONSE_SIGN[] = { "NORMAL POWER DOWN",
		"+CPSMSTATUS: \"EN", "+CPSMSTATUS: \"EX" };

#ifndef SUCCESS_RESPONSE_LENGTH
#define SUCCESS_RESPONSE_LENGTH (sizeof(SUCCESS_COMMAND_SIGN) / sizeof(SUCCESS_COMMAND_SIGN[0]))
#endif

#ifndef ERROR_RESPONSE_LENGTH
#define ERROR_RESPONSE_LENGTH (sizeof(ERROR_COMMAND_SIGN) / sizeof(ERROR_COMMAND_SIGN[0]))
#endif

#ifndef PASSIVE_RESPONSE_LENGTH
#define PASSIVE_RESPONSE_LENGTH (sizeof(PASSIVE_RESPONSE_SIGN) / sizeof(PASSIVE_RESPONSE_SIGN[0]))
#endif


#endif /* MODULE */
