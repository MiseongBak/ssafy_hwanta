<script setup>
import { GoogleMap, Marker } from 'vue3-google-map'
import { env } from '../stores/env'

const apiKey = env.apiKey
const patients = { lat: 37.238077, lng: 126.773033 } // 환자 위치
const destination = { lat: 37.238931, lng: 126.772997 } // 목적지 위치
</script>

<template>
  <main> 
    <div>
      <GoogleMap
        :api-key="apiKey"
        style="width: 75%; height: 500px"
        :center="patients"
        :zoom="17"
      >
        <Marker :options="{ position: patients }" />
        <Marker :options="{ position: destination}" />
      </GoogleMap>
    </div>
    <div></div>
  </main>
</template>

<style scoped>
</style>
