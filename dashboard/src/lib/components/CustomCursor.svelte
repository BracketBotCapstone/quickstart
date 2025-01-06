<script lang="ts">
  let { color, name } = $props<{ color: string, name: string }>();
</script>

<div 
  class="absolute pointer-events-none transform -translate-x-1/2 -translate-y-1/2 z-50"
  style:color={color}
>
  <svg 
    class="w-4 h-4 transform rotate-45" 
    viewBox="0 0 24 24" 
    fill="currentColor"
  >
    <path d="M5 2l14 14h-6v6l-2-2-2-2v-2h-6l2-14z" />
  </svg>
  <span class="ml-1 px-2 py-1 bg-black/75 text-white text-xs rounded-full whitespace-nowrap">
    {name}
  </span>
</div> 