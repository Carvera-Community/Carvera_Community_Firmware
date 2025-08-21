#ifndef COMMON_ENUMS_H
#define COMMON_ENUMS_H

// Common enums shared between modules to avoid duplicate definitions

// Packet parsing state enum used by SerialConsole and WifiProvider
enum ParseState { WAIT_HEADER, READ_LENGTH, READ_DATA, CHECK_FOOTER };

#endif // COMMON_ENUMS_H
