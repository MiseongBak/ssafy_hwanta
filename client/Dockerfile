FROM node:20.10.0

WORKDIR /usr/src/app

COPY ./package*.json ./

RUN npm install

COPY . /usr/src/app

EXPOSE 5173

CMD ["npm", "run", "dev"]