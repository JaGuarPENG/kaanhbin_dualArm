#!/bin/bash

VERSION="2.5.1.231219"

sudo rm -rf ./mr-release-${VERSION}
mkdir mr-release-${VERSION}
mkdir mr-release-${VERSION}/kaanh-project
sudo cp -rf /opt/poky/3.1.20/sysroots/aarch64-poky-linux/usr/aris ./mr-release-${VERSION}
sudo cp -rf /opt/poky/3.1.20/sysroots/aarch64-poky-linux/usr/kaanh ./mr-release-${VERSION}
sudo cp -rf /opt/poky/3.1.20/sysroots/aarch64-poky-linux/usr/kaanhbot ./mr-release-${VERSION}
sudo cp -rf ./kaanhbin/build/bin/kaanhbin ./mr-release-${VERSION}/kaanh-project/
sudo cp -rf ./kaanhbin/doc ./mr-release-${VERSION}/kaanh-project/
sudo chmod -R +666 *
zip -q -r mr-release-${VERSION}.zip mr-release-${VERSION}/



