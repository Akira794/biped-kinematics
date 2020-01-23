#ifndef _PLOT_H_
#define _PLOT_H_

#include "Link.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>

  class PlotLink
  {
    public:
      struct Link *ulink[LINK_NUM];
  		double angle[JOINT_NUM];
      FILE *gp;
  	public:
  		PlotLink(Link *ulink);
      ~PlotLink();
      void setFILEPtr(FILE *gp){ this->gp = gp;}
  /* plot-func-on gnuplot */
      double elm(Link *ulink, int root, int elem );
      void SetPlotConf(int, int );
      void PlotLeg(double, double, double, double );
  };

#endif
