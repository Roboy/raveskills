import requests
import json


CLIENT_ID = 'MY_CLIENT_ID'
SECRET = 'MY_SECRET_KEY'
ACCESS_TOKEN = 'MY_ACCESS_TOKEN'

TOKEN_URI = 'https://api.sandbox.paypal.com/v1/oauth2/token'
INVOICE_NEXT_ID_URI = 'https://api.sandbox.paypal.com/v2/invoicing/generate-next-invoice-number'
INVOICE_DRAFT_URI = 'https://api.sandbox.paypal.com/v2/invoicing/invoices'
INVOICE_LIST_URI = 'https://api.sandbox.paypal.com/v2/invoicing/invoices?total_required=true'
INVOICE_DELETE_BASE_URI = 'https://api.sandbox.paypal.com/v2/invoicing/invoices/'


def getAccessToken(client_id, secret):
    # Remove these 2 lines!
    client_id = CLIENT_ID
    secret = SECRET

    headers = {'Accept': 'application/json', 'Accept-Language': 'en_US'}
    data = {'grant_type': 'client_credentials'}
    
    auth = requests.post(url=TOKEN_URI, headers=headers, data=data, auth=requests.auth.HTTPBasicAuth(client_id, secret))
    
    if auth.status_code == requests.codes.ok:
        return auth.json()['access_token']
    else:
        # raise requests.HTTPError(auth.status_code)
        print('Could not get access token!')
        return None


def getNextInvoiceNumber(access_token):
    # Remove this line!
    access_token = ACCESS_TOKEN
    
    headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + str(access_token)}
    response = requests.post(url=INVOICE_NEXT_ID_URI, headers=headers)
    if response.status_code == requests.codes.ok:
        return response.json()['invoice_number']
    else:
        print('Could not get next invoice number!')
        return None


def createInvoiceDraft(access_token, price):
    # Remove this line!
    access_token = ACCESS_TOKEN
    invoice_number = getNextInvoiceNumber(access_token)
    if invoice_number is None:
        print('No invoice number!')
        return None
    else:
        print('Invoice number for the draft is', invoice_number)
        headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + str(access_token)}
        
        data = {
            'detail': {
                'invoice_number': str(invoice_number),
                'reference': 'deal-ref',
                'invoice_date': '2019-07-03',
                'currency_code': 'EUR',
                'note': 'Thank you for your business.',
                'term': 'No refunds.',
                'payment_term': {
                    'due_date': '2019-07-05'
                }
            },
            'invoicer': {
                'business_name': 'Roboy',
                'website': 'https://roboy.org',
                'address': {
                'address_line_1': 'TUM Entrepreneurship Center',
                'address_line_2': 'Lichtenbergstrasse 8',
                'postal_code': '85748',
                'admin_area_2': 'Garching',
                'admin_area_1': 'Munich',
                'country_code': 'DE'
                }
            },
            'items': [
                {
                    'name': 'Ice cream',
                    'description': 'Ice cream served by Roboy.',
                    'quantity': 1,
                    'unit_amount': {
                        'currency_code': 'EUR',
                        'value': str(price)
                    }
                }
            ]
            ,
            'configuration':{
                'partial_payment':{
                    'allow_partial_payment': 'false'
                },
                'allow_tip': 'true',
                'tax_calculated_after_discount': 'true',
                'tax_inclusive': 'true'
            },
            'amount':{
                'breakdown':{
                    'custom':{
                        'label': 'Roboy Ice Cream',
                        'amount':{
                            'currency_code': 'EUR',
                            'value': str(price)
                        }
                    }
                }
            }
        }
        data = json.dumps(data)
        response = requests.post(url=INVOICE_DRAFT_URI, headers=headers, data=data)
        if response.status_code == requests.codes.created:
            return response.json()['href']
        else:
            print('Could not create invoice draft!')
            return None


def getInvoiceList(access_token):
    # Remove this line!
    access_token = ACCESS_TOKEN
    
    headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + str(access_token)}
    response = requests.get(url=INVOICE_LIST_URI, headers=headers)
    if response.status_code == requests.codes.ok:
        response = response.json()
        if 'items' in response:
            return response['items']
        else:
            return response
    else:
        print('Could not get invoice list!')
        return None


def deleteInvoice(access_token, invoice_id):
    uri = INVOICE_DELETE_BASE_URI + str(invoice_id)
    headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + str(access_token)}
    
    response = requests.delete(url=uri, headers=headers)
    
    if response.status_code == requests.codes.no_content:
        return True
    else:
        print('Could not delete invoice!')
        return False


invoices = getInvoiceList(ACCESS_TOKEN)