#include "string_utils.h"

/// Compares two strings with no case check. Returns -1 if s1 < s2, 1 if s1 > s2, 0 if s1==s2
int cmp_nocase( const std::string& s1, const std::string&s2)
{
  std::string::const_iterator p1 = s1.begin();
  std::string::const_iterator p2 = s2.begin();

  while (p1 != s1.end() && p2 != s2.end()) {
    if ( toupper(*p1) != toupper(*p2) ) 
      return (toupper(*p1) < toupper(*p2)) ? -1 : 1;
    ++p1;
    p2++;
  }

  return (s2.size() == s1.size()) ? 0 : (s1.size() < s2.size()) ? -1 : 1; 
}

/// Removes leading and trailing blanks
std::string Trim( const std::string& str)
{
  std::string result(str);

  std::size_t startpos = result.find_first_not_of(" ");
  if (startpos != result.npos) // We found a blank
    result.erase(0, startpos);


  // Remove trailing blanks
  std::size_t blankpos = result.find_last_not_of(" "); // remove trailing blanks
  if (blankpos != result.npos) // We found a blank
    result.resize(blankpos+1);

  // A string with just blanks, remove the whole line
  if (startpos == result.npos && blankpos == result.npos)
    result.resize(0);

  return result;
}

/// Extracts a word from line, the word is removed from line
std::string GetWord( std::string& line )
{
  std::string word;
  std::size_t stop;

  line = line.substr(0, line.length()); // The word starts here
  line = Trim(line);

  stop = line.find_first_of(" "); // Find the next space, end of word
  if ( stop != line.npos ) {
    word = line.substr(0, stop);
    line = line.substr(stop, line.length());
    line = Trim(line);
  }

  // Is there anything left in the string?
  // that is, there could be only one word in the string
  else if ( line.length() ) {
    word = line;
    line.resize(0);
  }



  return word; 
}


/// Converts a string to a float, returns true for SUCCESS
bool StringToFloat(const std::string& word, float& val)
{
  float v;
  const char *cstr = word.c_str();

  //  for(int i = strlen(cstr) - 1; i >= 0; i--)
  //   if ( !(isdigit(cstr[i]) || cstr[i] == '.' ||cstr[i] == '-' || cstr[i] == '+' ||cstr[i] == 'e' ||cstr[i] == 'E'))
  //     return false;

  char *endptr;
  v = (float)strtod(cstr, &endptr);

  if (endptr ==cstr) return false;  /* no conversion */

  while (isspace((unsigned char)*endptr)) endptr++;
  if (*endptr != '\0') return false;  /* invalid trailing characters? */
  val = v;

  return true;
}


// return true if its float data
bool getData(std::string& str, std::string& key, std::string& s_data, float& f_data)
{
  std::string line = Trim(str);
  key = GetWord(line);

  float val;
  if (StringToFloat(line, val)) {
    f_data = val;
    return true;
  }
  s_data =line;
  return false;
}
