FROM alpine:3.7

ENV AWSCLI_VERSION "1.14.10"

RUN apk add --update \
    python \
    python-dev \
    py-pip \
    build-base \
    && pip install awscli==$AWSCLI_VERSION --upgrade \
    && apk --purge -v del py-pip \
    && rm -rf /var/cache/apk/* \
    && aws --version

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
