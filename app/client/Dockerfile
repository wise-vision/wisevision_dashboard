FROM node:16-alpine
WORKDIR /usr/src
COPY app /usr/src/app

WORKDIR /usr/src/app/client
RUN npm install

COPY . .

EXPOSE 3000

CMD ["npm", "start"]