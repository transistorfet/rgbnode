

#ifndef NERVESERIAL_H
#define NERVESERIAL_H

#include <Stream.h>

#define NS_BUFFER_SIZE		32
#define NS_MAX_ARGS		5

typedef struct {
	char *name;
	char args_min;
	void (*func)();
} NerveCommand_t;

class NerveSerial {
    private:
	Stream *m_serial;
	NerveCommand_t *m_commands = NULL;
	char m_rpos = 0;
	char m_avail = 0;
	char m_replied = 0;
	char m_read_buffer[NS_BUFFER_SIZE];
	char m_nargs = 0;
	char m_args[NS_MAX_ARGS];

    public:
	NerveSerial(Stream *serial);
	void set_commands(NerveCommand_t *commands);

	void check_read();
	int read();
	void clear();
	void process();

	inline char num_args() { return m_nargs; }
	inline char *get_arg(char num) { return &m_read_buffer[m_args[num]]; }
	inline char *get_command() { return m_read_buffer; }

	void start_reply();
	void print(char *str);
	void print_arg(char *str);
	void print(long num, char format);
	void print_arg(long num, char format);

	void send(char *fmt, ...);
};

/*
#define SERIAL_SPEED	19200

char serial_r = 0;
char serial_avail = 0;
char serial_rb[SERIAL_SIZE];

*/

#endif

