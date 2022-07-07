"""!@package botCallbacks
@file botCallbacks/deliver.py
@brief Contains all functions used in conversation with the bot with command /deliver.
"""

from telegram import ReplyKeyboardMarkup, ReplyKeyboardRemove, Update
from telegram.ext import (CallbackContext, ConversationHandler)
from telegram_bot_package.data.globaldata import *
import telegram_bot_package.data.globaldata as dt
import telegram_bot_package.rosDialog.publisher as pb
from telegram_bot_package.rosDialog.processData import *


def deliver(update: Update, context: CallbackContext):
    """!@brief Handles the conversation with the bot with command /deliver.
    Ask user for the pickup office.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: next state.
    """
    reply_keyboard = [[]]
    for office in dt.offices:
        reply_keyboard[0].append(office)
    update.message.reply_text("Great! Let's start a new delivery!!\nTell me the pick up office.",
                              reply_markup=ReplyKeyboardMarkup(
                                  reply_keyboard, one_time_keyboard=True, input_field_placeholder='What office?'
                              ))
    return PICKUP


def pickup(update: Update, context: CallbackContext) -> int:
    """!@brief Handles the conversation with the bot in the state PICKUP.
    Ask user for the destination office.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: next state.
    """
    requests[update.message.from_user.id] = {}
    requests[update.message.from_user.id]['pickup'] = update.message.text
    reply_keyboard = [[]]
    for office in dt.offices:
        if requests[update.message.from_user.id]['pickup'] != office:
            reply_keyboard[0].append(office)
    update.message.reply_text(
        'Thanks, '
        'where have I to bring it?',
        reply_markup=ReplyKeyboardMarkup(
            reply_keyboard, one_time_keyboard=True, input_field_placeholder='What office?'
        )
    )
    return DESTINATION


def destination(update: Update, context: CallbackContext) -> int:
    """!@brief Handles the conversation with the bot in the state DESTINATION.
    Ask user how many slots the package occupies.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: next state.
    """
    requests[update.message.from_user.id]['destination'] = update.message.text
    reply_keyboard = [[]]
    for n in range(1, slotsPerBot + 1):
        reply_keyboard[0].append(n)
    update.message.reply_text(
        'Great! Let\'s talk about space, \n'
        'tell me how many slots your package occupy. A slot is 20x20 cm. Write me a number, thanks!.',
        reply_markup=ReplyKeyboardMarkup(
            reply_keyboard, one_time_keyboard=True, input_field_placeholder='How many slots?'
        ),
    )
    return SLOTS


def slots(update: Update, context: CallbackContext) -> int:
    """!@brief Handles the conversation with the bot in the state SLOTS.
    Ask user what the priority of the mission is.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: next state.
    """
    requests[update.message.from_user.id]['slots'] = update.message.text
    reply_keyboard = [['High', 'Medium', 'Low']]
    update.message.reply_text(
        'Oh it\'s fine.\n'
        'What is the priority of your delivery?',
        reply_markup=ReplyKeyboardMarkup(
            reply_keyboard, one_time_keyboard=True, input_field_placeholder='Priority?'
        ),
    )
    return PRIORITY


def priority(update: Update, context: CallbackContext) -> int:
    """!@brief Handles the conversation with the bot in the state PRIORITY.
    Parse the previous input and give a feedback to the user.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: next state.
    """
    if update.message.text == 'High':
        requests[update.message.from_user.id]['priority'] = 3
    elif update.message.text == 'Medium':
        requests[update.message.from_user.id]['priority'] = 2
    elif update.message.text == 'Low':
        requests[update.message.from_user.id]['priority'] = 1
    else:
        requests[update.message.from_user.id]['priority'] = 1

    try:
        userRequest = processRequest(requests[update.message.from_user.id])
        pb.publisher.publishRequest(userRequest)
    except Exception as e:
        print(e)
        update.message.reply_text(
            'An error occurred!.\n'
            '{}\n'
            'Check the office name or try later!'.format(e),
            reply_markup=ReplyKeyboardRemove(),
        )
    else:
        update.message.reply_text(
            'Your request has been dealt with.\n'
            'The bot is coming!',
            reply_markup=ReplyKeyboardRemove(),
        )
    return ConversationHandler.END


def cancel(update: Update, context: CallbackContext) -> int:
    """!@brief Handles the conversation with the bot with command /cancel and terminate conversation.
    @param update: Update object.
    @param context: CallbackContext object.
    @return: end of the conversation.
    """
    del requests[update.message.from_user.id]
    update.message.reply_text(
        'Bye! I hope we can complete the delivery another day.', reply_markup=ReplyKeyboardRemove()
    )
    return ConversationHandler.END
