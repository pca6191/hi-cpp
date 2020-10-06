/*
 * agv_string_list.h
 *
 *  Created on: 2020年7月17日
 *      Author: kc.chang
 */

#ifndef SOURCE_DIRECTORY__LIB_AGV_COMMON_INCLUDE_AGV_STRING_LIST_H_
#define SOURCE_DIRECTORY__LIB_AGV_COMMON_INCLUDE_AGV_STRING_LIST_H_

#include <string>
#include <vector>

namespace agv {

class StringList {
public:
  StringList();
  ~StringList();

  /// 將另一個字串列表複製到本身
  void assign(StringList &sl);

  /// 從檔案中讀入字串
  bool load_from_file(std::string filename);

  /// 將內存的字串存入檔案, delim 為分隔字串或換行符號
  bool save_to_file(std::string filename, std::string delim = "");

  /// 將字串收集納入
  void add(std::string s);

  /// 將內存的字串清光
  void clear();

  /// 內存的字串列表
  std::vector<std::string> strings_;
};

}  // namespace ag

#endif /* SOURCE_DIRECTORY__LIB_AGV_COMMON_INCLUDE_AGV_STRING_LIST_H_ */
