#!/bin/bash


/usr/sbin/nginx -p "$(rospack find toc)" -c "$(rospack find toc)/conf/nginx.conf"
