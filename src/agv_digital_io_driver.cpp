/*
 * agv_fork_driver.cpp
 *
 *  Created on: 2019年12月17日
 *      Author: luther.wu
 */

#include <agv_digital_io_driver.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

namespace agv {

DigitalIODriver::DigitalIODriver()
{
}

DigitalIODriver::~DigitalIODriver()
{
}

bool DigitalIODriver::is_healthy()
{
  return healthy_;
}

void DigitalIODriver::fork_stop()
{
}

void DigitalIODriver::fork_up_by_fast_gear()
{
}

void DigitalIODriver::fork_up_by_slow_gear()
{
}

void DigitalIODriver::fork_down()
{
}

}  // namespace agv
