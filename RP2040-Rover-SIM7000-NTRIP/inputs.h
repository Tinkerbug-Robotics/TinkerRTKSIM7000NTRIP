// NTRIP caster host, requires IP address and not domain name currently
// TODO add DNS translation so web addresses can be used
char host[] = "3.143.243.81";

// NTRIP caster port (2101 is a typical default)
int http_port = 2101;

// Mount point or base station to receive data from
char mount_point[] = "XXXXXX";

// User name (leave blank as "" if not needed)
char user[] = "XXXXXXX";

// NTRIP caster password
char psw[] = "XXXXXXXX";

char src_string[]   = "";

// APN for mobile provider, for example "wholesale" for Tello
char apn[] = "hologram";

// Desired baud rate for talking to SIM7000
int sim7000_baud_rate = 19200;
