/****************************************************************************************************************************
  WebHandlers.cpp - Dead simple Ethernet AsyncWebServer.

  For W6100 LwIP Ethernet in ESP32 (ESP32 + W6100)

  AsyncWebServer_ESP32_W6100 is a library for the LwIP Ethernet W6100 in ESP32 to run AsyncWebServer

  Based on and modified from ESPAsyncWebServer (https://github.com/me-no-dev/ESPAsyncWebServer)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncWebServer_ESP32_W6100
  Licensed under GPLv3 license

  Original author: Hristo Gochkov

  Copyright (c) 2016 Hristo Gochkov. All rights reserved.

  This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License along with this library;
  if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Version: 1.6.4

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.6.4   K Hoang      09/01/2023 Initial porting for ESP32 + W6100. Sync with AsyncWebServer_ESP32_W5500 v1.6.4
 *****************************************************************************************************************************/

#include "AsyncWebServer_ESP32_W6100.h"

#include "WebHandlerImpl.h"

/////////////////////////////////////////////////

AsyncStaticWebHandler::AsyncStaticWebHandler(const char* uri, FS& fs, const char* path, const char* cache_control)
  : _fs(fs), _uri(uri), _path(path), _default_file("index.htm"), _cache_control(cache_control), _last_modified(""),
    _callback(nullptr)
{
  // Ensure leading '/'
  if (_uri.length() == 0 || _uri[0] != '/')
    _uri = "/" + _uri;

  if (_path.length() == 0 || _path[0] != '/')
    _path = "/" + _path;

  // If path ends with '/' we assume a hint that this is a directory to improve performance.
  // However - if it does not end with '/' we, can't assume a file, path can still be a directory.
  _isDir = _path[_path.length() - 1] == '/';

  // Remove the trailing '/' so we can handle default file
  // Notice that root will be "" not "/"
  if (_uri[_uri.length() - 1] == '/')
    _uri = _uri.substring(0, _uri.length() - 1);

  if (_path[_path.length() - 1] == '/')
    _path = _path.substring(0, _path.length() - 1);

  // Reset stats
  _gzipFirst = false;
  _gzipStats = 0xF8;
}

/////////////////////////////////////////////////

AsyncStaticWebHandler& AsyncStaticWebHandler::setIsDir(bool isDir)
{
  _isDir = isDir;

  return *this;
}

/////////////////////////////////////////////////

AsyncStaticWebHandler& AsyncStaticWebHandler::setDefaultFile(const char* filename)
{
  _default_file = String(filename);

  return *this;
}

/////////////////////////////////////////////////

AsyncStaticWebHandler& AsyncStaticWebHandler::setCacheControl(const char* cache_control)
{
  _cache_control = String(cache_control);

  return *this;
}

/////////////////////////////////////////////////

AsyncStaticWebHandler& AsyncStaticWebHandler::setLastModified(const char* last_modified)
{
  _last_modified = String(last_modified);

  return *this;
}

/////////////////////////////////////////////////

AsyncStaticWebHandler& AsyncStaticWebHandler::setLastModified(struct tm* last_modified)
{
  char result[30];
  strftime (result, 30, "%a, %d %b %Y %H:%M:%S %Z", last_modified);

  return setLastModified((const char *)result);
}

/////////////////////////////////////////////////

bool AsyncStaticWebHandler::canHandle(AsyncWebServerRequest *request)
{
  if (request->method() != HTTP_GET || !request->url().startsWith(_uri)
      || !request->isExpectedRequestedConnType(RCT_DEFAULT, RCT_HTTP) )
  {
    return false;
  }

  if (_getFile(request))
  {
    // We interested in "If-Modified-Since" header to check if file was modified
    if (_last_modified.length())
      request->addInterestingHeader("If-Modified-Since");

    if (_cache_control.length())
      request->addInterestingHeader("If-None-Match");

    AWS_LOGDEBUG("[AsyncStaticWebHandler::canHandle] TRUE");

    return true;
  }

  return false;
}

/////////////////////////////////////////////////

bool AsyncStaticWebHandler::_getFile(AsyncWebServerRequest *request)
{
  // Remove the found uri
  String path = request->url().substring(_uri.length());

  // We can skip the file check and look for default if request is to the root of a directory or that request path ends with '/'
  bool canSkipFileCheck = (_isDir && path.length() == 0) || (path.length() && path[path.length() - 1] == '/');

  path = _path + path;

  // Do we have a file or .gz file
  if (!canSkipFileCheck && _fileExists(request, path))
    return true;

  // Can't handle if not default file
  if (_default_file.length() == 0)
    return false;

  // Try to add default file, ensure there is a trailing '/' ot the path.
  if (path.length() == 0 || path[path.length() - 1] != '/')
    path += "/";

  path += _default_file;

  return _fileExists(request, path);
}

/////////////////////////////////////////////////

#define FILE_IS_REAL(f) (f == true && !f.isDirectory())

/////////////////////////////////////////////////

bool AsyncStaticWebHandler::_fileExists(AsyncWebServerRequest *request, const String& path)
{
  bool fileFound = false;
  bool gzipFound = false;

  String gzip = path + ".gz";

  if (_gzipFirst)
  {
    request->_tempFile = _fs.open(gzip, "r");
    gzipFound = FILE_IS_REAL(request->_tempFile);

    if (!gzipFound)
    {
      request->_tempFile = _fs.open(path, "r");
      fileFound = FILE_IS_REAL(request->_tempFile);
    }
  }
  else
  {
    request->_tempFile = _fs.open(path, "r");
    fileFound = FILE_IS_REAL(request->_tempFile);

    if (!fileFound)
    {
      request->_tempFile = _fs.open(gzip, "r");
      gzipFound = FILE_IS_REAL(request->_tempFile);
    }
  }

  bool found = fileFound || gzipFound;

  if (found)
  {
    // Extract the file name from the path and keep it in _tempObject
    size_t pathLen = path.length();
    char * _tempPath = (char*)malloc(pathLen + 1);
    snprintf(_tempPath, pathLen + 1, "%s", path.c_str());
    request->_tempObject = (void*)_tempPath;

    // Calculate gzip statistic
    _gzipStats = (_gzipStats << 1) + (gzipFound ? 1 : 0);

    if (_gzipStats == 0x00)
      _gzipFirst = false; // All files are not gzip
    else if (_gzipStats == 0xFF)
      _gzipFirst = true; // All files are gzip
    else
      _gzipFirst = _countBits(_gzipStats) > 4; // IF we have more gzip files - try gzip first
  }

  return found;
}

/////////////////////////////////////////////////

uint8_t AsyncStaticWebHandler::_countBits(const uint8_t value) const
{
  uint8_t w = value;
  uint8_t n;

  for (n = 0; w != 0; n++)
    w &= w - 1;

  return n;
}

/////////////////////////////////////////////////

void AsyncStaticWebHandler::handleRequest(AsyncWebServerRequest *request)
{
  // Get the filename from request->_tempObject and free it
  String filename = String((char*)request->_tempObject);
  free(request->_tempObject);
  request->_tempObject = NULL;

  if ((_username != "" && _password != "") && !request->authenticate(_username.c_str(), _password.c_str()))
    return request->requestAuthentication();

  if (request->_tempFile == true)
  {
    String etag = String(request->_tempFile.size());

    if (_last_modified.length() && _last_modified == request->header("If-Modified-Since"))
    {
      request->_tempFile.close();
      request->send(304); // Not modified
    }
    else if (_cache_control.length() && request->hasHeader("If-None-Match")
             && request->header("If-None-Match").equals(etag))
    {
      request->_tempFile.close();
      AsyncWebServerResponse * response = new AsyncBasicResponse(304); // Not modified

      response->addHeader("Cache-Control", _cache_control);
      response->addHeader("ETag", etag);
      request->send(response);
    }
    else
    {
      AsyncWebServerResponse * response = new AsyncFileResponse(request->_tempFile, filename, String(), false, _callback);

      if (_last_modified.length())
        response->addHeader("Last-Modified", _last_modified);

      if (_cache_control.length())
      {
        response->addHeader("Cache-Control", _cache_control);
        response->addHeader("ETag", etag);
      }

      request->send(response);
    }
  }
  else
  {
    request->send(404);
  }
}