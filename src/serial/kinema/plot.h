#ifndef __PLOT_H__
#define __PLOT_H__

#include "link.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>
#include "demo_biped_link.h"

using namespace Eigen;

/* plot-func-on gnuplot */
void SetPlotConf(FILE*,int, int );
void PlotLeg( FILE*, Link*, double, double, double, double );
#endif
