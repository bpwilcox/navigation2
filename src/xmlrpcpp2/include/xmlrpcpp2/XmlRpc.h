// this file modified by Morgan Quigley on 22 April 2008 to add 
// a std::exception-derived class
#ifndef _XMLRPC_H_
#define _XMLRPC_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
// 

#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <string>
#endif

#include "xmlrpcpp2/XmlRpcClient.h"
#include "xmlrpcpp2/XmlRpcException.h"
#include "xmlrpcpp2/XmlRpcServer.h"
#include "xmlrpcpp2/XmlRpcServerMethod.h"
#include "xmlrpcpp2/XmlRpcValue.h"
#include "xmlrpcpp2/XmlRpcUtil.h"

#include <stdexcept>

#endif // _XMLRPC_H_
