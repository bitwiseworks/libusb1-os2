/*
 * poll_windows: poll compatibility wrapper for OS/2
 * Copyright (C) 2013 Paul Smedley <paul@smedley.id.au>
 * poll() implementation from:
 * Greg Parker     gparker-web@sealiesoftware.com     December 2000
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#include "poll_os2.h"
#include <stdio.h>

int usbi_pipe(int filedes[2])
{
	int rc,fds[2];
	rc = socketpair(AF_UNIX, SOCK_STREAM,0, fds);
	filedes[0]=fds[0];
	filedes[1]=fds[1];
	return rc;
}

int usbi_poll(struct pollfd *pollSet, int pollCount, int pollTimeout)
{
    struct timeval tv;
    struct timeval *tvp;
    fd_set readFDs, writeFDs, exceptFDs;
    fd_set *readp, *writep, *exceptp;
    struct pollfd *pollEnd, *p;
    int selected;
    int result;
    int maxFD;

    if (!pollSet) {
        pollEnd = NULL;
        readp = NULL;
        writep = NULL;
        exceptp = NULL;
        maxFD = 0;
    } 
    else {
        pollEnd = pollSet + pollCount;
        readp = &readFDs;
        writep = &writeFDs;
        exceptp = &exceptFDs;

        FD_ZERO(readp);
        FD_ZERO(writep);
        FD_ZERO(exceptp);
        
        // Find the biggest fd in the poll set
        maxFD = 0;
        for (p = pollSet; p < pollEnd; p++) {
            if (p->fd > maxFD) maxFD = p->fd;
        }

        if (maxFD >= FD_SETSIZE) {
            // At least one fd is too big
            errno = EINVAL;
            return -1;
        }
        
        // Transcribe flags from the poll set to the fd sets
        for (p = pollSet; p < pollEnd; p++) {
            if (p->fd < 0) {
                // Negative fd checks nothing and always reports zero
            } else {
                if (p->events & POLLIN)  FD_SET(p->fd, readp);
                if (p->events & POLLOUT) FD_SET(p->fd, writep);
                if (p->events != 0)      FD_SET(p->fd, exceptp);
                // POLLERR is never set coming in; poll() always reports errors
                // But don't report if we're not listening to anything at all.
            }
        }
    }
        
    // poll timeout is in milliseconds. Convert to struct timeval.
    // poll timeout == -1 : wait forever : select timeout of NULL
    // poll timeout == 0  : return immediately : select timeout of zero
    if (pollTimeout >= 0) {
        tv.tv_sec = pollTimeout / 1000;
        tv.tv_usec = (pollTimeout % 1000) * 1000;
        tvp = &tv;
    } else {
        tvp = NULL;
    }
    
    
    selected = select(maxFD+1, readp, writep, exceptp, tvp);


    if (selected < 0) {
        // Error during select
        result = -1;
    } 
    else if (selected > 0) {
        // Select found something
        // Transcribe result from fd sets to poll set.
        // Also count the number of selected fds. poll returns the 
        // number of ready fds; select returns the number of bits set.
        int polled = 0;
        for (p = pollSet; p < pollEnd; p++) {
	    p->revents = 0;
            if (p->fd < 0) {
                // Negative fd always reports zero
            } else {
                if ((p->events & POLLIN)   &&  FD_ISSET(p->fd, readp)) {
                    p->revents |= POLLIN;
                }
                if ((p->events & POLLOUT)  &&  FD_ISSET(p->fd, writep)) {
                    p->revents |= POLLOUT;
                }
                if ((p->events != 0)       &&  FD_ISSET(p->fd, exceptp)) {
                    p->revents |= POLLERR;
                }

                if (p->revents) polled++;
            }
	}
        result = polled;
    }
    else {
	// selected == 0, select timed out before anything happened
        // Clear all result bits and return zero.
        for (p = pollSet; p < pollEnd; p++) {
            p->revents = 0;
        }
        result = 0;
    }

    return result;
}

