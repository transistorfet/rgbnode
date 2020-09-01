
#include "NerveSerial.h"

NerveSerial::NerveSerial(Stream *serial)
{
	m_serial = serial;
}

void NerveSerial::set_commands(NerveCommand_t *commands)
{
	m_commands = commands;
}

void NerveSerial::check_read()
{
	if (read()) {
		process();
		clear();
	}
}

int NerveSerial::read()
{
	int b;

	b = m_serial->read();
	if (b == -1)
		return 0;

	m_read_buffer[m_rpos] = b;
	if (b == '\n' || b == '\r') {
		//m_read_buffer[m_rpos + 1] = '\0';
		m_read_buffer[m_rpos] = '\0';
		m_avail = 1;
	}
	if (m_rpos < NS_BUFFER_SIZE)
		m_rpos++;

	return m_avail;
}

void NerveSerial::clear()
{
	m_avail = 0;
	m_rpos = 0;
	m_replied = 0;
}

void NerveSerial::process()
{
	char i;

	if (!m_avail)
		return;

	/*
	for (i = 0; i < m_rpos && m_read_buffer[i] != '\0'; i++)
		if (m_read_buffer[i] == '?')
			m_read_buffer[i] = '\0';

	m_nargs = 0;
	m_args[0] = ++i;
	for (; i < m_rpos && m_read_buffer[i] != '\0'; i++) {
		if (m_read_buffer[i] == '&') {
			m_read_buffer[i] = '\0';

		}
		else if (m_read_buffer[i] == '=') {
			m_args[m_nargs] = i + 1;
			if (++m_nargs >= NS_MAX_ARGS)
				break;
		}
	}
	*/

	m_nargs = 0;
	for (i = 0; i < m_rpos && m_read_buffer[i] != '\0'; i++) {
		if (m_read_buffer[i] == ' ') {
			m_read_buffer[i] = '\0';
			m_args[m_nargs] = i + 1;
			if (++m_nargs >= NS_MAX_ARGS)
				break;

		}
	}

	for (i = 0; m_commands[i].name != 0; i++) {
		if (!strcmp(m_read_buffer, m_commands[i].name)) {
			if (m_nargs >= m_commands[i].args_min) {
				(*m_commands[i].func)();
				// TODO reencode and send back
				start_reply();
				m_serial->write('\n');
				return;
			}
		}
	}

	if (m_commands[i].func) {
		(*m_commands[i].func)();
	}
}

void NerveSerial::start_reply()
{
	if (m_replied)
		return;
	m_replied = 1;
	m_serial->print(m_read_buffer);
}

void NerveSerial::print(char *str)
{
	m_serial->print(str);
}

void NerveSerial::print_arg(char *str)
{
	m_serial->write(' ');
	m_serial->print(str);
}

void NerveSerial::print(long num, char format)
{
	m_serial->print(num, format);
}

void NerveSerial::print_arg(long num, char format)
{
	m_serial->write(' ');
	m_serial->print(num, format);
}


void NerveSerial::send(char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	for (int i = 0; fmt[i] != '\0'; i++) {
		if (fmt[i] == '%') {
			switch (fmt[i + 1]) {
				case 's': {
					char *str = va_arg(args, char *);
					m_serial->print(str);
					break;
				}
				case 'x': {
					int num = va_arg(args, int);
					m_serial->print(num, HEX);
					break;
				}
				case '%': {
					m_serial->write('%');
					break;
				}
				default:
					break;
			}
			i++;
		}
		else
			m_serial->write(fmt[i]);
	}

	va_end(args);
}
 
