#!/bin/bash

set -x -e

tar -xzf spinnaker-2.0.0.147-amd64-pkg.tar.gz
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinnaker_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinnaker-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinnaker-c_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinnaker-c-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinvideo_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinvideo-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinvideo-c_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/libspinvideo-c-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinview-qt_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinview-qt-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinupdate_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinupdate-dev_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinnaker_*.deb
sudo dpkg -i spinnaker-2.0.0.147-amd64/spinnaker-doc_*.deb