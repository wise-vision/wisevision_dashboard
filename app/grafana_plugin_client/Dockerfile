FROM node:20 AS builder

WORKDIR /usr/src/app
COPY package*.json ./
RUN npm ci

COPY . .
RUN npm run build

FROM grafana/grafana-enterprise:12.2.0-ubuntu

COPY --from=builder /usr/src/app/dist /opt/wisevision-wiseos-app

COPY provisioning /etc/grafana/provisioning

ENV GF_DEFAULT_APP_MODE=production \
    GF_PLUGINS_ALLOW_LOADING_UNSIGNED_PLUGINS=wisevision-wiseos-app \
    GF_LOG_LEVEL=info \
    GF_LOG_MODE=console \
    GF_PATHS_PLUGINS=/var/lib/grafana/plugins

EXPOSE 3000

USER root

ENTRYPOINT ["bash", "-c", "\
  mkdir -p /var/lib/grafana/plugins && \
  rm -rf /var/lib/grafana/plugins/wisevision-wiseos-app && \
  cp -r /opt/wisevision-wiseos-app /var/lib/grafana/plugins/wisevision-wiseos-app && \
  chown -R 472:472 /var/lib/grafana/plugins && \
  exec /run.sh \
"]