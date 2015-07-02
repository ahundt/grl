/*
Copyright (c) 2014 Shingo W. Kagami. All rights reserved.

 http://code.google.com/p/stattimer/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

/**
 @file stattimer.hpp
 @brief A stopwatch timer library that reports statistics
 @author Shingo W. Kagami
*/

#ifndef _STATTIMER_H_
#define _STATTIMER_H_

#include <math.h>
#include <string.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

/**
 *  Struct that a STimerReporterFunc function receives as an argument.
 */
struct STimerRecords {
    int id;             ///< Id number of the timer
    std::string label;  ///< Label of the timer
    double mean;        ///< mean of elapsed time [s]
    double stddev;      ///< standard deviation of elapsed time [s]
    double maximum;     ///< maximum of elapsed time [s]
    double minimum;     ///< minimum of elapsed time [s]
    std::vector<double> timebuf; ///< vector containing the recent values of elapsed time
    int nsample;        ///< number of samples used to compute the above statistics
};

/**
 * Type of a user-defined reporter function.
 *
 * @param rec [in] Timer statistics record
 */
typedef std::string (*STimerReporterFunc)(STimerRecords& rec);

// forward declarations
class STimer;
template<typename T> class STimerList_;
template<typename T> class STimerScoped_;

/**
 * See template class STimerList_. 
 */
typedef STimerList_<STimer> STimerList;

/**
 * See template class STimerScoped_. 
 */
typedef STimerScoped_<STimer> STimerScoped;

/**
 *  Class for a single timer.  This is used by STimerList class and will
 *  not be used by a user directly.
 */
class STimer {
    friend class STimerList_<STimer>;
public:
    STimer() : start_count(0.0), nsample(0), 
               dsum(0.0), dsqrsum(0.0), dmax(0.0), dmin(0.0),
               idx_timebuf(-1), is_recording(false) {}
    ~STimer() {}
protected:
    void start() {
        start_count = tickCount();
        is_recording = true;
    }
    void start(double start_count_value) {
        start_count = start_count_value;
        is_recording = true;
    }
    double stop() {
        double stop_count = tickCount();
        double delta = (stop_count - start_count) * tickPeriod();
        registerTime(delta);
        return stop_count; // used in laptime()
    }
    void registerTime(double delta) {
        if (is_recording) {
            dsum += delta;
            dsqrsum += delta * delta;
            if (nsample == 0) {
                dmax = dmin = delta;
            } else {
                if (delta > dmax) {
                    dmax = delta;
                }
                if (delta < dmin) {
                    dmin = delta;
                }
            }
            if (idx_timebuf >= 0) {
                timebuf.at(idx_timebuf) = delta;
                idx_timebuf++;
                if (idx_timebuf >= (int)timebuf.size()) {
                    idx_timebuf = 0;
                }
            }
            nsample++;
            is_recording = false;
        }
    }
    std::string report(STimerReporterFunc fn = NULL) const {
        if (fn == NULL) {
            fn = reporter_func;
        }
        if (nsample == 0 || fn == NULL) {
            std::string empty_str;
            return empty_str;
        }
        STimerRecords rec;
        calcStat(rec);
        return fn(rec);
    }
    void calcStat(STimerRecords& rec) const {
        rec.id = id;
        rec.label = label;
        rec.mean = dsum / nsample;
        double var = dsqrsum / nsample - rec.mean * rec.mean;
        rec.stddev = var < 0 ? 0.0 : sqrt(var);
        rec.maximum = dmax;
        rec.minimum = dmin;
        rec.nsample = nsample;
        if (idx_timebuf >= 0) {
            int bufsize = (int)timebuf.size();
            if (timebuf.at(idx_timebuf) < 0) {
                rec.timebuf.assign(idx_timebuf, 0);
                for (int i = 0; i < idx_timebuf; i++) {
                    rec.timebuf.at(i) = timebuf.at(i);
                }
            } else {
                rec.timebuf.assign(bufsize, 0);
                for (int i = idx_timebuf; i < bufsize; i++) {
                    rec.timebuf.at(i - idx_timebuf) = timebuf.at(i);
                }
                for (int i = 0; i < idx_timebuf; i++) {
                    rec.timebuf.at(bufsize - idx_timebuf + i) = timebuf.at(i);
                }
            }
        }
    }
    void setLabel(std::string l) {
        label = l;
    }
    void setReporterFunc(STimerReporterFunc f) {
        reporter_func = f;
    }
    void initTimeBuf(int len) {
        timebuf.assign(len, -1.0);
        idx_timebuf = 0;
    }
    void setId(int i) {
        id = i;
    }
    inline static double tickCount();
    inline static double tickPeriod();

    int id;
    double start_count;
    int nsample;
    double dsum;
    double dsqrsum;
    double dmax;
    double dmin;
    std::vector<double> timebuf;
    int idx_timebuf;
    bool is_recording;
    std::string label;
    STimerReporterFunc reporter_func;
};

/**
 *  Class that contains and manages multiple timers.
 * 
 *  This is designed as a template class just for testability (i.e. to
 *  cope with a mock subclass of STimer).  Users should always use
 *  STimerList, which is typedef of STimerList_<STimer>.
 */
template<typename TimerType>
class STimerList_ {
public:
    /**
     * Constructor that generates n timers.
     * @param n [in] number of timers initially generated
     * @param rfunc [in] reporter function, which defaults to reporterDefault
     * @param outfile [in] output file name, which defaults to stdout
     */
    STimerList_(int n, const STimerReporterFunc rfunc = reporterDefault,
                const std::string& outfile = "")
        : ntimer(n), timer(ntimer), outfile(outfile) {
        init(rfunc);
    }
    /** Constructor that generates ntimer_default (= 20) timers.
     * @param rfunc [in] reporter function, which defaults to reporterDefault
     * @param outfile [in] output file name, which defaults to stdout
     */
    STimerList_(STimerReporterFunc const rfunc = reporterDefault,
                const std::string& outfile = "")
        : ntimer(ntimer_default), timer(ntimer), outfile(outfile) {
        init(rfunc);
    }
    /** Destructor that calls report() and put the result to stdout.  */
    ~STimerList_() {
        if (outfile == "") {
            std::cout << report();
        } else {
            std::ofstream ofs(outfile);
            ofs << report();
        }
    }

    /** 
     * Start the timer whose id = i.  If the timer does not exist, it
     * is automatially generated.
     *
     * @param i [in] Timer id
     */
    inline void start(int i) {
        if (i >= ntimer) { expand(i + 1); }
        timer.at(i).start();
    }
    /** 
     * Start the timer with the specified label.  If the timer does
     * not exist, it is automatially generated.
     * 
     * @param label [in] Timer label
     * @return Timer id associated to the label
     */
    inline int start(const char *label) {
        int i = findLabel(label);
        timer.at(i).start();
        return i;
    }

    /** 
     * Stop the timer whose id = i and update the statistics record.
     * If the timer has not been started, nothing is done.  If the
     * timer does not exist, it is automatially generated (and of
     * course nothing is recorded).
     *
     * @param i [in] Timer id
     */
    inline void stop(int i) {
        if (i >= ntimer) { expand(i + 1); }
        timer.at(i).stop();
    }

    /** 
     * Stop the timer with the specified label.  See description of
     * stop(int).
     *
     * @param label [in] Timer label
     * @return Timer id associated to the label
     */
    inline int stop(const char *label) {
        int i = findLabel(label);
        timer.at(i).stop();
        return i;
    }

    /** 
     * Record the lap time of the timer whose id = i.  If the timer
     * does not exist, it is automatially generated.  It is equivalent
     * to calling stop(i) followed by start(i) with zero delay.
     *
     * @param i [in] Timer id
     */
    inline void laptime(int i) {
        if (i >= ntimer) { expand(i + 1); }
        double t = timer.at(i).stop();
        timer.at(i).start(t);
    }
    /** 
     * Record the lap time of the timer with the specified label.  See
     * description of laptime(int).
     * 
     * @param label [in] Timer label
     * @return Timer id associated to the label
     */
    inline int laptime(const char *label) {
        int i = findLabel(label);
        double t = timer.at(i).stop();
        timer.at(i).start(t);
        return i;
    }
    /**
     * Initialize the buffer to record recent values of elapsed time
     * of the timer whose id = i.  If the timer does not exist, it is
     * automatially generated.
     *
     * After this is called, the timer starts recording the elapsed
     * time between each start() and stop() (or consecutive laptime())
     * in addition to updating the statistics data.  Only the recent
     * len values at most are preserved.  The recorded values can be
     * accessed in the STimerReporterFunc function as rec.timebuf
     * vector.
     *
     * @param i [in] Timer id
     * @param len [in] Length of the buffer
     */
    void initTimeBuf(int i, int len) {
        if (i >= ntimer) { expand(i + 1); }
        timer.at(i).initTimeBuf(len);
    }
    /**
     * Initialize the buffer to record recent values of elapsed time
     * of the timer with the specified label.  See description of
     * initTimeBuf(i, len).
     * 
     * @param label [in] Timer label
     * @param len [in] Length of the buffer
     * @return Timer id associated to the label
     */
    int initTimeBuf(const char *label, int len) {
        int i = findLabel(label);
        timer.at(i).initTimeBuf(len);
        return i;
    }
    /**
     * Set the label to the timer whose id = i.  If the same label
     * already exists (for another timer), that old label is cleared
     * in advance.  If the timer i does not exist, it is
     * automatically generated.  If the timer i already has a label,
     * the old label is cleared in advance.
     *
     * @param i [in] Timer id to which the label is set. 
     * @param label [in] Timer label
     */
    void setLabel(int i, const char *label) {
        std::string label_str(label);
        if (labelExists(label_str)) {
            int old_i = label_assoc[label_str];
            clearLabel(old_i);
        }
        if (i >= ntimer) {
            expand(i + 1);
        } else if (hasLabel(i)) {
            clearLabel(i);
        }
        doSetLabel(i, label_str);
    }

    /**
     * Set the reporter function to the timer i. If the timer does not
     * exist, it is automatially generated.  The reporter functions of
     * the other timers are unchanged.
     *
     * @param i [in] Timer id to which the reporter function is associated
     * @param fn [in] Reporter function.  When fn = NULL, nothing is reported.
     */
    void setReporterFunc(int i, STimerReporterFunc fn) {
        if (i >= ntimer) { expand(i + 1); }
        timer.at(i).setReporterFunc(fn);
    }
    /**
     * Set the reporter function to the timer specified with the label. If the timer does not
     * exist, it is automatially generated.  The reporter functions of
     * the other timers are unchanged.
     *
     * @param label [in] Label of the timer to which the reporter function is associated
     * @param fn [in] Reporter function.  When fn = NULL, nothing is reported.
     * @return Timer id associated to the label
     */
    int setReporterFunc(const char *label, STimerReporterFunc fn) {
        int i = findLabel(label);
        timer.at(i).setReporterFunc(fn);
        return i;
    }
    /**
     * Set the reporter function to all the timers.  This reporter
     * function is also applied to the timers that will be generated
     * in future.
     *
     * @param fn [in] Reporter function.  When fn = NULL, nothing is reported.
     */
    void setReporterFunc(STimerReporterFunc fn) {
        common_reporter_func = fn;
        for (int i = 0; i < ntimer; i++) {
            timer.at(i).setReporterFunc(fn);
        }
    }
    /**
     * Generate the report of the timer i and return it as string.
     * Reporter function to be used can optionally be specified.  If
     * the timer i does not exist, it is automatically generated (and
     * an empty string is reported).
     * 
     * @param i [in] Timer id
     * @param fn [in] Reporter function. If not specified, the
     * function set by setReporterFunc is used.
     * @return Generated report
     */
    std::string report(int i, STimerReporterFunc fn = NULL) {
        if (i >= ntimer) { expand(i + 1); }
        return timer.at(i).report(fn);
    }
    /**
     * Generate the report of the timer with the specified label and
     * return it as string.  See description of report(i, fn).
     *
     * @param label [in] Timer label
     * @param fn [in] Reporter function. If not specified, the
     * function set by setReporterFunc is used.
     * @return Generated report
     */
    std::string report(const char *label, STimerReporterFunc fn = NULL) {
        int i = findLabel(label);
        return timer.at(i).report(fn);
    }
    /**
     * Generate the reports of all the timer and return the
     * concatenated string of the reports.  Reporter function to be
     * used can optionally be specified.
     *
     * @param fn [in] Reporter function. If not specified, the
     * function set by setReporterFunc is used for each timer.
     * @return Generated report
     */
    std::string report(STimerReporterFunc fn = NULL) const {
        std::ostringstream stream;
        for (int i = 0; i < (int)timer.size(); i++) {
            stream << timer.at(i).report(fn);
        }
        return stream.str();
    }

    /**
     * Generate and return STimerRecords object of the timer whose id
     * = i.  If the timer i does not exist, it is automatically
     * generated (and an empty records are returned).  This may be
     * useful when a user wants the statistics in a form other than
     * string.
     * 
     * @param i [in] Timer id
     * @return Generated record object
     */
    STimerRecords calcStat(int i) {
        if (i >= ntimer) { expand(i + 1); }
        STimerRecords rec;
        timer.at(i).calcStat(rec);
        return rec;
    }
    /**
     * Generate and return STimerRecords object of the timer 
     * with the specified label.  See description of calcStat(i).
     * 
     * @param label [in] Timer label
     * @return Generated record object
     */
    STimerRecords calcStat(const char *label) {
        int i = findLabel(label);
        STimerRecords rec;
        timer.at(i).calcStat(rec);
        return rec;
    }

    static std::string reporterDefault(STimerRecords& rec) {
        std::ostringstream stream;
        if (!rec.label.empty()) {
            stream << rec.label << ":" << std::endl;
        } else {
            stream << rec.id << ":" << std::endl;
        }
        stream << std::fixed << std::setprecision(3)
               << " mean = " << 1000.0 * rec.mean
               << " [ms], std = " << 1000.0 * rec.stddev
               << ", max/min = " << 1000.0 * rec.maximum
               << "/" << 1000.0 * rec.minimum
               << "; " << rec.nsample
               << " iter." << std::endl;
        return stream.str();
    }

    static std::string reporterTSV(STimerRecords& rec) {
        std::ostringstream stream;
        stream << rec.id << "\t"
               << "\"" << rec.label << "\"\t"
               << std::fixed << std::setprecision(3)
               << 1000.0 * rec.mean << "\t"
               << 1000.0 * rec.stddev << "\t"
               << 1000.0 * rec.maximum << "\t"
               << 1000.0 * rec.minimum << "\t"
               << rec.nsample << std::endl;
        return stream.str();
    }

private:
    void init(const STimerReporterFunc rfunc) {
        common_reporter_func = rfunc;
        for (int i = 0; i < ntimer; i++) {
            timer.at(i).setId(i);
            timer.at(i).setReporterFunc(rfunc);
        }
    }
    inline void clearLabel(int i) {
        label_assoc.erase(timer.at(i).label);
        timer.at(i).setLabel("");
    }
    inline void doSetLabel(int i, std::string& label) {
        timer.at(i).setLabel(label);
        label_assoc[label] = i;
    }
    inline void expand(int new_ntimer) {
        timer.resize(new_ntimer, TimerType());
        for (int i = ntimer; i < new_ntimer; i++) {
            timer.at(i).setId(i);
            timer.at(i).setReporterFunc(common_reporter_func);
        }
        ntimer = new_ntimer;
    }
    inline int findLabel(const char *label) {
        std::string label_str(label);
        std::map<std::string, int>::iterator it = label_assoc.find(label_str);
        if (it == label_assoc.end()) {
            expand(ntimer + 1);
            doSetLabel(ntimer - 1, label_str);
        }
        return label_assoc[label_str];
    }
    bool labelExists(const std::string& label) const {
        return label_assoc.find(label) != label_assoc.end();
    }
    bool hasLabel(int i) const {
        return label_assoc.find(timer.at(i).label) != label_assoc.end();
    }

    static const int ntimer_default = 20;
    int ntimer;
    std::vector<TimerType> timer;
    std::map<std::string, int> label_assoc;
    STimerReporterFunc common_reporter_func;
    std::string outfile;
};

/**
 * Class for a scoped timer object.  It makes a specified timer to
 * measure the elapsed time between the time when it is constructed
 * and the time when it is destructed.
 *
 *  This is designed as a template class just for testability (i.e. to
 *  cope with a mock subclass of STimer).  Users should always use
 *  STimerScoped, which is typedef of STimerScoped_<STimer>.
 *
 */
template <typename TimerType>
class STimerScoped_ {
public:
    /**
     * Constructor that starts the timer i in the STimerList stlist. 
     *
     * @param stlist [in] STimerList that contains the timer to be used
     * @param i [in] Timer id
     */
    STimerScoped_(STimerList_<TimerType>& stlist, int i) : st(stlist), idx(i) {
        st.start(idx);
    }
    /**
     * Constructor that starts the timer with specified label in the
     * STimerList stlist.
     *
     * @param stlist [in] STimerList that contains the timer to be used
     * @param label [in] Timer label
     */
    STimerScoped_(STimerList_<TimerType>& stlist, const char *label)
        : st(stlist) {
        idx = st.start(label);
    }
    /**
     * Destructor in which the timer is stopped. 
     */
    ~STimerScoped_() {
        st.stop(idx);
    }
private:
    STimerList_<TimerType>& st;
    int idx;
};



/* Derived from OpenCV (core/src/system.cpp) */
#if defined WIN32 || defined _WIN32 || defined WINCE
#include <windows.h>
double STimer::tickCount() {
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart;
}
double STimer::tickPeriod() {
    static double tick_period = 0.0;
    if (tick_period == 0.0) {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        tick_period = 1.0 / (double)freq.QuadPart;
    }
    return tick_period;
}

#elif (defined __linux || defined __linux__) && defined CLOCK_MONOTONIC_RAW
#include <time.h>
double STimer::tickCount() {
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (double)((long long int)tp.tv_sec * 1000000000 + tp.tv_nsec);
}
double STimer::tickPeriod() {
    return 1e-9;
}

#elif defined __MACH__ && defined __APPLE__
#include <mach/mach_time.h>
double STimer::tickCount() {
    return (double)mach_absolute_time();
}
double STimer::tickPeriod() {
    static double tick_period = 0.0;
    if (tick_period == 0.0) {
        mach_timebase_info_data_t sTimebaseInfo;
        mach_timebase_info(&sTimebaseInfo);
        tick_period = sTimebaseInfo.numer / sTimebaseInfo.denom * 1e-9;
    }
    return tick_period;
}

#else // other Unix-like
#include <sys/time.h>
double STimer::tickCount() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    return (double)(tv.tv_sec * 1000000 + tv.tv_usec);
}
double STimer::tickPeriod() {
    return 1e-6;
}
#endif

#endif /* _STATTIMER_H_ */
