#ifndef ROTORS_CONTROL_CELL
#define ROTORS_CONTROL_CELL

#include <math.h>           // abs
#include <tuple>
#include <string>

#include "rotors_control/common.h"

namespace rotors_control {


class Cell {
 public:
  Cell();
  Cell(std::tuple<int, int, int> newTpl);
  Cell(double x, double y, double z);
  Cell(double x, double y);
  Cell(geometry_msgs::Point point);
  Cell(Eigen::Vector3d point);

  // Get the indices of the Cell
  int x() const;
  int y() const;
  int z() const;

  // Get the coordinates of the center-point of the Cell 
  double xPos() const;
  double yPos() const;
  double zPos() const;

  double manhattanDist(double _x, double _y, double _z) const;
  double distance2D(const Cell & b) const;
  double diagDistance2D(const Cell & b) const;
  double angle() const;
  
  Cell getNeighborFromYaw(double yaw) const;
  std::vector<Cell> getFlowNeighbors() const;
  std::vector<Cell> getDiagonalNeighbors() const;

  std::string asString() const;
  
  // Member variables
  std::tuple<int, int, int> tpl;
  static constexpr double scale = 1.0;
};

inline bool operator==(const Cell & lhs, const Cell & rhs) {return lhs.tpl == rhs.tpl;}
inline bool operator!=(const Cell & lhs, const Cell & rhs) {return !operator==(lhs,rhs);}
inline bool operator< (const Cell & lhs, const Cell & rhs) {return lhs.tpl < rhs.tpl;}
inline bool operator> (const Cell & lhs, const Cell & rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Cell & lhs, const Cell & rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Cell & lhs, const Cell & rhs) {return !operator< (lhs,rhs);}

inline Cell operator+(const Cell& lhs, const Cell& rhs) {
  Cell res(std::tuple<int, int, int>(lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z()));
  return res;
}
inline Cell operator-(const Cell& lhs, const Cell& rhs) {
  Cell res(std::tuple<int, int, int>(lhs.x() - rhs.x(), lhs.y() - rhs.y(), lhs.z() - rhs.z()));
  return res;
}

typedef std::pair<Cell, double> CellDistancePair;

struct HashCell {
    size_t operator()(const Cell & cell ) const
    {
        return std::hash<std::string>()(cell.asString());
    }
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_CELL
