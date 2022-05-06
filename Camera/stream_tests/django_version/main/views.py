from django.shortcuts import render
from .models import ToDoList, Item
# Create your views here.


def index(response):
    return render(response, "main/stream.html", {})


def home(responce):
    pass
