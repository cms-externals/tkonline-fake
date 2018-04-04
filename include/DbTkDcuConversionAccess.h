/*
  This file is part of Fec Software project.

  Fec Software is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Fec Software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Fec Software; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  Copyright 2002 - 2003, Damien VINTACHE - IReS/IN2P3

*/


#ifndef DBTKDCUCONVERSIONACCESS_H
#define DBTKDCUCONVERSIONACCESS_H
#include "DbAccess.h"

// ostream use for int to string conversion 
#include <sstream>
#include "stringConv.h"

//declaration of the exception handler for the DCU
#include "FecExceptionHandler.h"



/** \brief This class is implemented to handle the communication between the DCU supervisor software and the database.
 *
 */
class DbTkDcuConversionAccess : public DbCommonAccess {

 public:
  //
  // public functions
  //
  /** \brief Default constructor
   */
  DbTkDcuConversionAccess(bool threaded = false) noexcept(false);

  /** \brief Constructor with connection parameters
   */
  DbTkDcuConversionAccess (std::string user, std::string passwd, std::string dbPath, bool threaded = false) noexcept(false);

  /** \brief Destructor
   */
  ~DbTkDcuConversionAccess ( ) noexcept(false);

  /** \brief Download a Clob from database
   */
  oracle::occi::Clob *getXMLClob(tscType32 dcuHardId ) noexcept(false);
  oracle::occi::Clob *getXMLClob(std::string partitionName) noexcept(false);

  /** \brief Upload a Clob to the database for configuration
   */
  void setXMLClob (std::string buffer) noexcept(false);
};

#endif
