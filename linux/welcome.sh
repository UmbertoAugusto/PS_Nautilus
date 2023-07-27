#!/bin/bash

echo -e "Bem vindo $USER ao terminal do $HOSTNAME"

curl wttr.in/?0

echo "Bem vindo $USER ao terminal do $HOSTNAME">>~/.welcome.data
echo "$(curl wttr.in/?0)">>~/.welcome.data

