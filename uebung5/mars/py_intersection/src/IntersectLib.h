#include <vector>
#include <string>

class IntersectLib {
public:
  struct Line {
    double x1, y1, x2, y2;
  };

  IntersectLib();
  ~IntersectLib() {}
  
  static IntersectLib* il;
  std::string message;
  std::vector<Line> walls;

  double getIntersect(const Line &l1, const Line &l2);
  double getIntersect(const Line &l);
};
