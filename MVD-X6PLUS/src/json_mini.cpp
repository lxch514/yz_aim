// 最小 JSON 值类型的实现。
#include "mvd/json_mini.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace mj {
namespace {

void escape(std::string& out, const std::string& s) {
  out.push_back('"');
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04X", c);
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out.push_back('"');
}

class Parser {
 public:
  explicit Parser(const std::string& t) : t_(t) {}

  bool run(Value* out) {
    skip();
    if (!value(out)) return false;
    skip();
    return true;
  }

 private:
  void skip() {
    while (i_ < t_.size()) {
      char c = t_[i_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++i_;
      } else {
        break;
      }
    }
  }

  bool literal(const char* word) {
    size_t n = std::strlen(word);
    if (t_.compare(i_, n, word) != 0) return false;
    i_ += n;
    return true;
  }

  bool value(Value* out) {
    if (i_ >= t_.size()) return false;
    char c = t_[i_];
    if (c == '{') return object(out);
    if (c == '[') return array(out);
    if (c == '"') {
      std::string s;
      if (!string(&s)) return false;
      *out = Value::string(std::move(s));
      return true;
    }
    if (literal("null")) {
      *out = Value();
      return true;
    }
    if (literal("true")) {
      *out = Value::boolean(true);
      return true;
    }
    if (literal("false")) {
      *out = Value::boolean(false);
      return true;
    }
    return number(out);
  }

  bool object(Value* out) {
    ++i_;  // '{'
    *out = Value::object();
    skip();
    if (i_ < t_.size() && t_[i_] == '}') {
      ++i_;
      return true;
    }
    while (i_ < t_.size()) {
      skip();
      std::string key;
      if (!string(&key)) return false;
      skip();
      if (i_ >= t_.size() || t_[i_] != ':') return false;
      ++i_;
      skip();
      Value v;
      if (!value(&v)) return false;
      out->set(key, std::move(v));
      skip();
      if (i_ < t_.size() && t_[i_] == ',') {
        ++i_;
        continue;
      }
      if (i_ < t_.size() && t_[i_] == '}') {
        ++i_;
        return true;
      }
      return false;
    }
    return false;
  }

  bool array(Value* out) {
    ++i_;  // '['
    *out = Value::array();
    skip();
    if (i_ < t_.size() && t_[i_] == ']') {
      ++i_;
      return true;
    }
    while (i_ < t_.size()) {
      skip();
      Value v;
      if (!value(&v)) return false;
      out->push(std::move(v));
      skip();
      if (i_ < t_.size() && t_[i_] == ',') {
        ++i_;
        continue;
      }
      if (i_ < t_.size() && t_[i_] == ']') {
        ++i_;
        return true;
      }
      return false;
    }
    return false;
  }

  bool string(std::string* out) {
    if (i_ >= t_.size() || t_[i_] != '"') return false;
    ++i_;
    out->clear();
    while (i_ < t_.size()) {
      char c = t_[i_++];
      if (c == '"') return true;
      if (c != '\\') {
        out->push_back(c);
        continue;
      }
      if (i_ >= t_.size()) return false;
      char e = t_[i_++];
      switch (e) {
        case '"': out->push_back('"'); break;
        case '\\': out->push_back('\\'); break;
        case '/': out->push_back('/'); break;
        case 'b': out->push_back('\b'); break;
        case 'f': out->push_back('\f'); break;
        case 'n': out->push_back('\n'); break;
        case 'r': out->push_back('\r'); break;
        case 't': out->push_back('\t'); break;
        case 'u': {
          if (i_ + 4 > t_.size()) return false;
          unsigned cp = static_cast<unsigned>(std::strtoul(t_.substr(i_, 4).c_str(), nullptr, 16));
          i_ += 4;
          if (cp >= 0xD800 && cp <= 0xDBFF && i_ + 6 <= t_.size() && t_[i_] == '\\' &&
              t_[i_ + 1] == 'u') {
            unsigned lo = static_cast<unsigned>(std::strtoul(t_.substr(i_ + 2, 4).c_str(), nullptr, 16));
            i_ += 6;
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          }
          if (cp < 0x80) {
            out->push_back(static_cast<char>(cp));
          } else if (cp < 0x800) {
            out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          } else if (cp < 0x10000) {
            out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          } else {
            out->push_back(static_cast<char>(0xF0 | (cp >> 18)));
            out->push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          }
          break;
        }
        default: return false;
      }
    }
    return false;
  }

  bool number(Value* out) {
    size_t start = i_;
    if (i_ < t_.size() && (t_[i_] == '-' || t_[i_] == '+')) ++i_;
    bool digits = false;
    while (i_ < t_.size()) {
      char c = t_[i_];
      if ((c >= '0' && c <= '9')) {
        digits = true;
        ++i_;
      } else if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
        ++i_;
      } else {
        break;
      }
    }
    if (!digits) return false;
    *out = Value::number(std::strtod(t_.substr(start, i_ - start).c_str(), nullptr));
    return true;
  }

  const std::string& t_;
  size_t i_ = 0;
};

}  // namespace

Value Value::boolean(bool v) {
  Value out;
  out.type_ = BOOL;
  out.bool_ = v;
  return out;
}

Value Value::number(double v) {
  Value out;
  out.type_ = NUM;
  out.num_ = v;
  return out;
}

Value Value::string(std::string v) {
  Value out;
  out.type_ = STR;
  out.str_ = std::move(v);
  return out;
}

Value Value::array() {
  Value out;
  out.type_ = ARR;
  return out;
}

Value Value::object() {
  Value out;
  out.type_ = OBJ;
  return out;
}

long long Value::as_int(long long def) const {
  if (type_ != NUM) return def;
  return static_cast<long long>(std::llround(num_));
}

Value& Value::set(const std::string& key, Value v) {
  if (type_ != OBJ) type_ = OBJ;
  for (auto& kv : obj_) {
    if (kv.first == key) {
      kv.second = std::move(v);
      return kv.second;
    }
  }
  obj_.emplace_back(key, std::move(v));
  return obj_.back().second;
}

const Value* Value::find(const std::string& key) const {
  if (type_ != OBJ) return nullptr;
  for (const auto& kv : obj_) {
    if (kv.first == key) return &kv.second;
  }
  return nullptr;
}

Value& Value::push(Value v) {
  if (type_ != ARR) type_ = ARR;
  arr_.push_back(std::move(v));
  return arr_.back();
}

std::size_t Value::size() const {
  if (type_ == ARR) return arr_.size();
  if (type_ == OBJ) return obj_.size();
  return 0;
}

std::string Value::dump(int indent) const {
  std::string out;
  dump_to(out, indent, 0);
  return out;
}

void Value::dump_to(std::string& out, int indent, int depth) const {
  switch (type_) {
    case NUL:
      out += "null";
      return;
    case BOOL:
      out += bool_ ? "true" : "false";
      return;
    case NUM:
      out += num_to_str(num_);
      return;
    case STR:
      escape(out, str_);
      return;
    case ARR: {
      if (arr_.empty()) {
        out += "[]";
        return;
      }
      out.push_back('[');
      for (size_t i = 0; i < arr_.size(); ++i) {
        if (i) out.push_back(',');
        if (indent >= 0) {
          out.push_back('\n');
          out.append(static_cast<size_t>(indent) * (depth + 1), ' ');
        }
        arr_[i].dump_to(out, indent, depth + 1);
      }
      if (indent >= 0) {
        out.push_back('\n');
        out.append(static_cast<size_t>(indent) * depth, ' ');
      }
      out.push_back(']');
      return;
    }
    case OBJ: {
      if (obj_.empty()) {
        out += "{}";
        return;
      }
      out.push_back('{');
      for (size_t i = 0; i < obj_.size(); ++i) {
        if (i) out.push_back(',');
        if (indent >= 0) {
          out.push_back('\n');
          out.append(static_cast<size_t>(indent) * (depth + 1), ' ');
        }
        escape(out, obj_[i].first);
        out.push_back(':');
        if (indent >= 0) out.push_back(' ');
        obj_[i].second.dump_to(out, indent, depth + 1);
      }
      if (indent >= 0) {
        out.push_back('\n');
        out.append(static_cast<size_t>(indent) * depth, ' ');
      }
      out.push_back('}');
      return;
    }
  }
}

std::string num_to_str(double v) {
  if (!std::isfinite(v)) return "null";
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.12g", v);
  return std::string(buf);
}

Value parse(const std::string& text, bool* ok) {
  Parser p(text);
  Value out;
  bool good = p.run(&out);
  if (ok) *ok = good;
  return good ? out : Value();
}

}  // namespace mj
