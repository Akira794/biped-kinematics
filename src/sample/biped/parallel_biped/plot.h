#ifndef __PLOT_H__
#define __PLOT_H__

#include "Link.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>

using namespace Eigen;

/* plot-func-on gnuplot */
void SetPlotConf(FILE*,int, int );
void PlotLeg( FILE*, Link*, double, double, double, double );
#endif
