#include <Python.h>
//#include <PythonInterpreter.hpp>
#include "IntersectLib.h"
#include <sstream>

static PyObject *
intersect_system(PyObject *self, PyObject *args) {
    const char *command;
    int sts;

    if (!PyArg_ParseTuple(args, "s", &command))
        return NULL;
    PyObject *r = Py_BuildValue("s", IntersectLib::il->message.c_str());
    IntersectLib::il->message = "";
    return r;
}

static PyObject *
intersect_get_intersection(PyObject *self, PyObject *args) {
    double sts;

    // TODO parse list of four values
    IntersectLib::Line l;
    if (!PyArg_ParseTuple(args, "dddd", &l.x1, &l.y1, &l.x2, &l.y2))
        return NULL;

    double v = IntersectLib::il->getIntersect(l);
    
    PyObject *r = Py_BuildValue("d", v);
    return r;
}

static PyMethodDef IntersectMethods[] = {
   {"get_intersect",  intersect_get_intersection, METH_VARARGS,
     "Get intersect."},
   {"system",  intersect_system, METH_VARARGS,
     "Execute a shell command."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initintersect(void) {
    (void) Py_InitModule("intersect", IntersectMethods);
    new IntersectLib();
}

IntersectLib* IntersectLib::il = NULL;

IntersectLib::IntersectLib() {
	char linebuf[255];
	Line new_line;
  std::stringstream ss;

	FILE* wallfile = fopen("walls.txt", "r");
	if (wallfile) {
		while (fgets(linebuf, 255, wallfile)) {
			if (!(linebuf[0] == '#')) {
				if (sscanf(linebuf, "%lf %lf %lf %lf\n", &new_line.x1, &new_line.y1,
                   &new_line.x2, &new_line.y2)) {
					walls.push_back(new_line);
				} else {
          ss << "wall file line %i not parsable: " << walls.size() << linebuf << std::endl;
				}
			}
		}
		fclose(wallfile);
	} else {
    ss << "unable to open walls file" << std::endl;
	}
  ss << "loaded " << walls.size() << " walls" << std::endl;
  message = ss.str();
  IntersectLib::il = this;
}

double IntersectLib::getIntersect(const Line &l1, const Line &l2) {
  double xa2a1 = l1.x2-l1.x1;
  double ya2a1 = l1.y2-l1.y1;
  double ya1b1 = l1.y1-l2.y1;
  double xa1b1 = l1.x1-l2.x1;
  double yb2b1 = l2.y2-l2.y1;
  double xb2b1 = l2.x2-l2.x1;

  double d = ((yb2b1*xa2a1)-(xb2b1*ya2a1));
  if( d == 0 ) return 2;
  d = 1./d;
  double u1 = ((xb2b1*ya1b1)-(yb2b1*xa1b1))*d;
  double u2 = ((xa2a1*ya1b1)-(ya2a1*xa1b1))*d;
  //fprintf(stderr, " (%g|%g)", u1, u2);
  if(u1 < 0 || u2 < 0 || u1 > 1.0 || u2 > 1.0) return 2;
  //fprintf(stderr, "#%g", u1);
  return u1;
}

double IntersectLib::getIntersect(const Line &l) {
  std::vector<Line>::iterator it = walls.begin();
  double min = 2;
  double v;
  for(; it!=walls.end(); ++it) {
    v = getIntersect(l, *it);
    if(min < 0 || min > v) min = v;
  }
  //fprintf(stderr, "%d %g\n", (int)walls.size(), min);
  return min;
}
