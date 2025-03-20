#!/bin/bash

gcc symc.c -o symc -lX11 -lGL -lXrandr -lm -I../cadigo/src && ./symc
