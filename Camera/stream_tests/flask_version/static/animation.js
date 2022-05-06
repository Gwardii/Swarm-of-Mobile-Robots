const hamburger = document.querySelector(".menu-btn");
const navLinks = document.querySelector(".nav-links");
const links = document.querySelectorAll(".nav-links li");
const nav = document.querySelector("nav");

hamburger.addEventListener("click", () =>{
    navLinks.classList.toggle("open");
    links.forEach(link => {
        link.classList.toggle('fade');
    });
    nav.classList.toggle('active');
    
});