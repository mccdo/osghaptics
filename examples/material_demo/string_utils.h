#ifndef __StringUtils_h__
#define __StringUtils_h__

#include <string>
/// Compares two strings with no case check. Returns -1 if s1 < s2, 1 if s1 > s2, 0 if s1==s2
int cmp_nocase( const std::string& s1, const std::string&s2);

/// Removes leading and trailing blanks
std::string Trim( const std::string& str);

/// Extracts a word from line, the word is removed from line
std::string GetWord( std::string& line );

/// Converts a string to a float, returns true for SUCCESS
bool StringToFloat(const std::string& word, float& val);

// return true if its float data
bool getData(std::string& str, std::string& key, std::string& s_data, float& f_data);

#endif
