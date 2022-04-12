# jupiter-fan-control

playground for fan control PID tuning ideation

## Install
If you have fakeroot and arch devel tools installed, the process looks like this:
### install.sh
        #! /bin/bash
        echo "building"
        makepkg -g >> PKGBUILD
        makepkg -scif
        echo "restarting service"
	sudo systemctl enable jupiter-fan-control.service
        sudo systemctl daemon-reload
        sudo systemctl restart jupiter-fan-control.service
        echo "finished"

The above script can be used to rebuild and restart the service if you're tinkering with it.

Full instructions can be found here:
[jupiter-fan-control install instructions](https://jupiterhw.atlassian.net/wiki/spaces/DVE/pages/325648385/Setup+jupiter-fan-control+Driver+testing)

## Checking the output
#### Service status:

        sudo systemctl status jupiter-fan-control.service

#### Fancontrol output watch:

        sudo journalctl -u jupiter-fan-control.service -f

## Modifying the config
Config file is stored in /etc/jupiter-fan-control-config.yaml

	vim /etc/jupiter-fan-control-config.yaml
	sudo systemctl restart jupiter-fan-control.service

Config file is stored in /etc/jupiter-fan-control-config.yaml

	vim /etc/jupiter-fan-control-config.yaml
	sudo systemctl restart weld-fancontrol.service

