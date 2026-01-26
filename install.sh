#!/usr/bin/env bash

# WARNING: 12GB
# Euroc MAV Stereo + IMU Dataset
# About: https://www.research-collection.ethz.ch/server/api/core/bitstreams/d861e63b-cfa9-4411-85a5-5ad6b3526e44/content 
# Landing Page: https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f 

# Download
urls=(
    https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download
    # https://www.research-collection.ethz.ch/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/download
    # https://www.research-collection.ethz.ch/bitstreams/ea12bc01-3677-4b4c-853d-87c7870b8c44/download
    # https://www.research-collection.ethz.ch/bitstreams/5732e864-10f1-49e7-befb-669ee29ff770/download
)

mkdir data/euroc_datasets
cd data/euroc_datasets

for url in "${urls[@]}"; do
    fname=$(basename "$url")
    echo "Downloading $fname…"
    curl -L "$url" -o "$fname"
done

# Unzip
names=(
    machine_hall.zip
)

for name in "${names[@]}"; do
    echo "Unzipping $name"
    unzip "$name"
done

unzip machine_hall/MH_01_easy/MH_01_easy.zip