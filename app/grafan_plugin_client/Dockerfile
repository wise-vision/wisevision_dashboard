FROM node:20 AS builder

WORKDIR /usr/src/app
COPY package*.json ./
RUN npm ci

COPY . .
RUN npm run build

FROM grafana/grafana-enterprise:12.2.0-ubuntu

COPY --from=builder /usr/src/app/dist /var/lib/grafana/plugins/wisevision-wiseos-app

COPY provisioning /etc/grafana/provisioning

ENV GF_DEFAULT_APP_MODE=development \
    GF_PLUGINS_ALLOW_LOADING_UNSIGNED_PLUGINS=wisevision-wiseos-app \
    GF_LOG_LEVEL=debug \
    GF_LOG_MODE=console

EXPOSE 3000
