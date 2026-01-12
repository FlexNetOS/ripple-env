// Extra JavaScript for ripple-env documentation

// Copy button notification enhancement
document.addEventListener('DOMContentLoaded', function() {
  // Add smooth scrolling for anchor links
  document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
      e.preventDefault();
      const target = document.querySelector(this.getAttribute('href'));
      if (target) {
        target.scrollIntoView({
          behavior: 'smooth',
          block: 'start'
        });
      }
    });
  });
});

// Keyboard shortcuts
document.addEventListener('keydown', function(e) {
  // Press '/' to focus search
  if (e.key === '/' && !e.ctrlKey && !e.metaKey) {
    const search = document.querySelector('.md-search__input');
    if (search && document.activeElement !== search) {
      e.preventDefault();
      search.focus();
    }
  }
});
