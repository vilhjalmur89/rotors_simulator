#ifndef ROTORS_CONTROL_NODE
#define ROTORS_CONTROL_NODE

#include <string>

#include "rotors_control/cell.h"
#include "rotors_control/common.h"

namespace rotors_control {
  
class Node {
 public:
  Node() = default ;
  Node(const Cell cell, const Cell parent)
      : cell(cell), parent(parent) {
  }
  Cell cell;
  Cell parent;
};

inline bool operator==(const Node& lhs, const Node& rhs) {
  return lhs.cell == rhs.cell && lhs.parent == rhs.parent;}
inline bool operator< (const Node& lhs, const Node& rhs) {
  return lhs.cell < rhs.cell || (lhs.cell == rhs.cell && lhs.parent < rhs.parent);}
inline bool operator!=(const Node& lhs, const Node& rhs) {return !operator==(lhs,rhs);}
inline bool operator> (const Node& lhs, const Node& rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Node& lhs, const Node& rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Node& lhs, const Node& rhs) {return !operator< (lhs,rhs);}

typedef std::pair<Node, double> NodeDistancePair;

struct HashNode {
  size_t operator()(const Node &node ) const {
    HashCell hash;
    std::string s = "";
    s += hash(node.cell);
    s += " ";
    s += hash(node.parent);
    return std::hash<std::string>()(s);
  }
};  

class CompareDist {
 public:
  bool operator()(const CellDistancePair n1, const CellDistancePair n2) {
    return n1.second > n2.second;
  }
  bool operator()(const NodeDistancePair n1, const NodeDistancePair n2) {
    return n1.second > n2.second;
  }
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_NODE
