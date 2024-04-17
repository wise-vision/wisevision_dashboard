# Use an official Node runtime as a parent image
FROM node:14-slim

# Set the working directory
WORKDIR /usr/src/app

# Install app dependencies
COPY package*.json ./
RUN npm install

# Bundle app source
COPY . .

# Expose port and start application
EXPOSE 3000
CMD ["npm", "start"]
