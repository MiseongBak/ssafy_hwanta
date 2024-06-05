// stores/rescueRequestStore.js
import { defineStore } from 'pinia'
import axios from 'axios'

export const useRescueRequestStore = defineStore('rescueRequest', {
  state: () => ({
    requests: [], // 여기에 rescue requests 데이터를 저장합니다
  }),
  actions: {
    async fetchRequests() {
      try {
        const response = await axios.get('여기에_API_주소_입력')
        this.requests = response.data // API로부터 받은 데이터를 저장
      } catch (error) {
        console.error('Rescue Request Fetch Error:', error)
      }
    },
  },
})
