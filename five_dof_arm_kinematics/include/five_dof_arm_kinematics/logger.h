/*
 * logger.h
 *
 *  Created on: 31-Aug-2016
 *      Author: rohin
 */

#ifndef INCLUDE_LOGGER_H_
#define INCLUDE_LOGGER_H_

#include <string>

namespace five_dof_arm_kinematics
{

class Logger
{
    public:

        Logger();

        virtual ~Logger();

        /**
         * Write a log message. The default implementation is a dummy which does
         * nothing. This function should be overridden in a derived class to
         * achieve different behavior.
         *
         * @param msg The message to write to the log.
         *
         * @param file The source file from where the message was logged.
         *
         * @param line The line in the file from where the message was logged.
         */

        virtual void write(const std::string &msg, const char *file, int line);


    public:
        /**
         * A null object which can be used as a default argument.
         */
        static Logger null;
};

}



#endif /* INCLUDE_LOGGER_H_ */
