// 最小 JSON 值类型：解析 ip 命令输出、生成日志与统计文件。
#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace mj {

class Value {
 public:
  enum Type { NUL = 0, BOOL, NUM, STR, ARR, OBJ };

  Value() = default;

  static Value boolean(bool v);
  static Value number(double v);
  static Value string(std::string v);
  static Value array();
  static Value object();

  Type type() const { return type_; }
  bool is_null() const { return type_ == NUL; }
  bool is_obj() const { return type_ == OBJ; }
  bool is_arr() const { return type_ == ARR; }

  bool as_bool(bool def = false) const { return type_ == BOOL ? bool_ : def; }
  double as_num(double def = 0.0) const { return type_ == NUM ? num_ : def; }
  long long as_int(long long def = 0) const;
  const std::string& as_str() const { return str_; }

  // 对象成员读写，键不存在时返回 nullptr / 自动插入。
  Value& set(const std::string& key, Value v);
  const Value* find(const std::string& key) const;
  bool has(const std::string& key) const { return find(key) != nullptr; }

  // 数组元素追加。
  Value& push(Value v);
  const std::vector<Value>& items() const { return arr_; }
  const std::vector<std::pair<std::string, Value>>& entries() const { return obj_; }
  std::size_t size() const;

  std::string dump(int indent = -1) const;

 private:
  void dump_to(std::string& out, int indent, int depth) const;

  Type type_ = NUL;
  bool bool_ = false;
  double num_ = 0.0;
  std::string str_;
  std::vector<Value> arr_;
  std::vector<std::pair<std::string, Value>> obj_;
};

// 解析一段 JSON 文本；失败时返回空值并把 ok 置 false。
Value parse(const std::string& text, bool* ok = nullptr);

// 数字转字符串，整数值不带小数点。
std::string num_to_str(double v);

}  // namespace mj
