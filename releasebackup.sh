#!/bin/bash

VERSION="2.5.1.240105"

sudo rm -rf ./release-${VERSION}
mkdir release-${VERSION}
mkdir release-${VERSION}/kaanh-project
sudo cp -rf /usr/aris ./release-${VERSION}
sudo cp -rf /usr/kaanh ./release-${VERSION}
sudo cp -rf /usr/kaanhbot ./release-${VERSION}
sudo cp -rf ./kaanhbin/build/bin/kaanhbin ./release-${VERSION}/kaanh-project/
sudo cp -rf ./kaanhbin/doc ./release-${VERSION}/kaanh-project/
sudo chmod -R +666 *
zip -q -r release-${VERSION}.zip release-${VERSION}



