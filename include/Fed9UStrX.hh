/*
 * The Apache Software License, Version 1.1
 * 
 * Copyright (c) 2002 The Apache Software Foundation.  All rights
 * reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 
 * 3. The end-user documentation included with the redistribution,
 *    if any, must include the following acknowledgment:  
 *       "This product includes software developed by the
 *        Apache Software Foundation (http://www.apache.org/)."
 *    Alternately, this acknowledgment may appear in the software itself,
 *    if and wherever such third-party acknowledgments normally appear.
 * 
 * 4. The names "Xerces" and "Apache Software Foundation" must
 *    not be used to endorse or promote products derived from this
 *    software without prior written permission. For written 
 *    permission, please contact apache\@apache.org.
 * 
 * 5. Products derived from this software may not be called "Apache",
 *    nor may "Apache" appear in their name, without prior written
 *    permission of the Apache Software Foundation.
 * 
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE APACHE SOFTWARE FOUNDATION OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * ====================================================================
 * 
 * This software consists of voluntary contributions made by many
 * individuals on behalf of the Apache Software Foundation, and was
 * originally based on software copyright (c) 1999, International
 * Business Machines, Inc., http://www.ibm.com .  For more information
 * on the Apache Software Foundation, please see
 * <http://www.apache.org/>.
 */

#ifndef _Fed9UStrX_H_
#define _Fed9UStrX_H_

//Check if using Xerces version 1
#if XERCES > 1

//Check Xerces version
#include "xercesc/util/XercesVersion.hpp"
#if _XERCES_VERSION >= 20300

#include<iostream>
//using.*std::ostream;

#include <xercesc/util/XMLString.hpp>
//using xercesc::XMLString;

/**
 * \brief This is a simple class that lets us do easy (though not terribly efficient) trancoding of XMLCh data to local code page for display.
 */
class Fed9UStrX
{
public :

  /** \name Constructors and Destructor */
  //@{

  /**
   * \brief Constructor.
   * \param toTranscode A constant point to constant data of the XMLCh to decode to a normal C string.
   *
   * Takes an XMLCh and converts that to a C char*. The string in this form can be retrieved using localForm().
   */
  Fed9UStrX(const XMLCh* const toTranscode)
  {
    // Call the private transcoding method
    fLocalForm = xercesc::XMLString::transcode(toTranscode);
  }

  /**
   * \brief Destructor.
   *
   * Deletes the C string that the XMLCh was converted to.
   */
  ~Fed9UStrX()
  {
    delete [] fLocalForm;
  }
  //@}

  /** \name Getter methods */
  //@{

  /**
   * \brief  Return the C string of the XMLCh that was passed at construction.
   * \return const char* A read only pointer to the C string.
   */
  const char* localForm() const
    {
        return fLocalForm;
    }

private :

  char*   fLocalForm; //!< This is the local code page form of the string.
};

/**
 * \brief  Overload of the stream operator to write a C string made by the Fed9UStrX class to an output stream.
 * \param  target Output stream the string is to be written to.
 * \param  toDump Fed9UStrX object that is to be written to the stream.
 * \return ostream& Reference to the output stream the data was written to.
 */
inline std::ostream& operator<<(std::ostream& target, const Fed9UStrX& toDump)
{
    target << toDump.localForm();
    return target;
}

#endif

#endif

#endif
