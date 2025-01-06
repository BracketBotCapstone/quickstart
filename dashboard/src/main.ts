import './app.css'
import { mount } from 'svelte'
import App from './routes/+page.svelte'

mount(App, {
  target: document.getElementById('app')!
})