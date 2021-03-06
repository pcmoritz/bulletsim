/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2011 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef __OSGWTOOLS_VERSION_H__
#define __OSGWTOOLS_VERSION_H__ 1

#include "osgwTools/Export.h"
#include <osg/Version>
#include <string>


namespace osgwTools {


/** \defgroup Version Version utilities */
/*@{*/

// Please keep in sync with top-level CMakeLists.txt OSGWORKS_VERSION variable.
#define OSGWORKS_MAJOR_VERSION 2
#define OSGWORKS_MINOR_VERSION 0
#define OSGWORKS_SUB_VERSION 0

/** \brief osgWorks version number as an integer

C preprocessor integrated version number
The form is Mmmss, where:
   \li M is the major version
   \li mm is the minor version (zero-padded)
   \li ss is the sub version (zero padded)

Use this in version-specific code, for example:
\code
   #if( OSGWORKS_VERSION < 10500 )
      ... code specific to releases before v1.05
   #endif
\endcode
*/
#define OSGWORKS_VERSION ( \
        ( OSGWORKS_MAJOR_VERSION * 10000 ) + \
        ( OSGWORKS_MINOR_VERSION * 100 ) + \
        OSGWORKS_SUB_VERSION )

/** \brief OSG version number as an integer

This macro provides the same functionality as 
OSGWORKS_VERSION, but instead encodes the OSG version
number as an integer.
OSG didn't provide a useful compile-time version
comparison macro until after the 2.9.6 and 2.8.2 releases.
\see OSGWORKS_VERSION */
#define OSGWORKS_OSG_VERSION ( \
        ( OPENSCENEGRAPH_MAJOR_VERSION * 10000 ) + \
        ( OPENSCENEGRAPH_MINOR_VERSION * 100 ) + \
        OPENSCENEGRAPH_PATCH_VERSION )

/** \brief Run-time access to the osgWorks version number

Returns OSGWORKS_VERSION, the osgWorks version number as an integer
\see OSGWORKS_VERSION
*/
unsigned int OSGWTOOLS_EXPORT getVersionNumber();

/** \brief osgWorks version number as a string

Example:
\code
osgWorks version 1.1.0 (10100)
\endcode
*/
std::string OSGWTOOLS_EXPORT getVersionString();

/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_VERSION_H__
#endif
