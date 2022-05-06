#!/bin/bash
echo "stopping service"
sudo systemctl stop jupiter-fan-control.service
echo "packaging"
makepkg -scif
echo "restarting service"
sudo systemctl enable jupiter-fan-control.service
sudo systemctl daemon-reload
sudo systemctl restart jupiter-fan-control.service
echo "finished"