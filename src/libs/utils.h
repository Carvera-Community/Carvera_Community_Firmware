#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include "time.h"

std::string lc(const std::string& str);

bool is_alpha( int );
bool is_digit( int );
bool is_numeric( int );
bool is_alphanum( int );
bool is_whitespace( int );

std::vector<std::string> split(const char *str, char c = ',');
std::vector<float> parse_number_list(const char *str);
std::vector<uint32_t> parse_number_list(const char *str, uint8_t radix);

std::string remove_non_number( std::string str );

uint16_t get_checksum(const std::string& to_check);
uint16_t get_checksum(const char* to_check);

void get_checksums(uint16_t check_sums[], const std::string& key);

std::string shift_parameter( std::string &parameters );

std::string get_arguments( const std::string& possible_command );

bool file_exists( const std::string file_name );

void system_reset( bool dfu= false );

std::string absolute_from_relative( std::string path );
std::string change_to_md5_path( std::string origin );
std::string change_to_lz_path( std::string origin );
void check_and_make_path( std::string origin );

int append_parameters(char *buf, std::vector<std::pair<char,float>> params, size_t bufsize);
std::string wcs2gcode(int wcs);
void safe_delay_us(uint32_t delay);
void safe_delay_ms(uint32_t delay);

#define confine(value, min, max) (((value) < (min))?(min):(((value) > (max))?(max):(value)))

struct tm *get_fftime(unsigned short t_date, unsigned short t_time, struct tm *timeinfo);

void ltrim(std::string& s, const char* t = " \t\n\r\f\v");


//#define dd(...) LPC_GPIO2->FIODIR = 0xffff; LPC_GPIO2->FIOCLR = 0xffff; LPC_GPIO2->FIOSET = __VA_ARGS__
