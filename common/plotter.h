#ifndef PLOTTER_H
#define PLOTTER_H

#include "thirdParty/tabulate.hpp"
#include <easylogging++.h>
#include <sstream>
#include <chrono>

using namespace tabulate;
class plotter{
public:
    plotter() = delete;
    ~plotter() = delete;

    static void  computeAllRunTime(const double elapsed_time_secs,
                                   const double elapsed_time_msecs){

        Table format;

        std::stringstream  string_s;
        std::stringstream  string_ms;
        string_s  << "all time s is : " << std::fixed << std::setprecision(3)
              << elapsed_time_secs   << " s " ;
        string_ms << "all time ms is : " << std::fixed << std::setprecision(3)
               << elapsed_time_msecs  << " ms"  ;
        format.add_row({string_s.str(),string_ms.str()});

        format.format().border_color(Color::red);
        format
        .format()
        .border_color(Color::red)
        .font_color(Color::white)
        .font_style({FontStyle::bold})
        .padding_top(0)
        .padding_bottom(0);
        LOG(INFO) << "\n" << format << "\n";

    }

};
#endif // PLOTTER_H
