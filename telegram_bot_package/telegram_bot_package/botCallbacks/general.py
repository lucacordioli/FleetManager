"""!@package botCallbacks
@file botCallbacks/general.py
@brief Contains all functions used in a general conversation.
"""

from telegram import Update
from telegram.ext import CallbackContext, ConversationHandler
from telegram_bot_package.data.globaldata import requests


def start(update: Update, context: CallbackContext):
    """!@brief Handles the conversation with the bot with command /start and give a welcome message to user.
    @param update: Update object.
    @param context: CallbackContext object.
    """
    name = update.message.from_user.first_name
    reply = "{}, welcome to LogicMove!\nYou can use /deliver to pick up a package and deliver it to another office!".format(
        name)


def help(update: Update, context: CallbackContext):
    """!@brief Handles the conversation with the bot with command /help.
    @param update: Update object.
    @param context: CallbackContext object.
    """
    update.message.reply_text("Use /deliver to pick up a package and deliver it to another office!")


def unknown(update: Update, context: CallbackContext):
    """!@brief Handles the conversation with the bot with unknown command.
    @param update: Update object.
    @param context: CallbackContext object.
    """
    update.message.reply_text(
        "Sorry '%s' is not a valid command" % update.message.text)


def unknown_text(update: Update, context: CallbackContext):
    """!@brief Handles the conversation with the bot with unknown text.
    @param update: Update object.
    @param context: CallbackContext object.
    """
    update.message.reply_text(
        "Sorry I can't recognize you , you said '%s'" % update.message.text)
