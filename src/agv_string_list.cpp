/*
 * agv_string_list.cpp
 *
 *  Created on: 2020年7月17日
 *      Author: kcchang
 */
#include <fstream>
#include <agv_string_list.h>


namespace agv {

StringList::StringList()
{
}

StringList::~StringList()
{
}

void StringList::assign(StringList &sl)
{
  strings_ = sl.strings_;
}

bool StringList::load_from_file(std::string filename)
{
  std::ifstream fin(filename.c_str());

  if (fin.fail()) {
    return false;
  }

  std::string line;

  // 讀入的 line, 會去掉 '\n'
  while (!getline(fin, line).eof()) {
    strings_.push_back(line);
  }

  fin.close();
  return true;
}

bool StringList::save_to_file(std::string filename, std::string delim)
{
  std::ofstream fout(filename.c_str());

  if (fout.fail()) {
    return false;
  }

  if (strings_.size() == 0) {
    fout.close();
    return true;
  }

  for (unsigned int i = 0; i < strings_.size(); i++) {
    std::string s = strings_[i];
    if (i == 0) {
      fout.write(s.c_str(), s.size());
      continue;
    }
    s = delim + s;
    fout.write(s.c_str(), s.size());
  }

  fout.close();
  return true;
}

void StringList::add(std::string s)
{
  strings_.push_back(s);
}

void StringList::clear()
{
  strings_.clear();
}

}  // namespace agv


