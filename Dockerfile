# https://hub.docker.com/_/ubuntu/tags
FROM ubuntu:22.04

RUN apt update && \
    apt install build-essential python3 python3-pip python3-venv xsltproc libxml2-utils -y --no-install-recommends

WORKDIR /project
ENTRYPOINT ["make"]
