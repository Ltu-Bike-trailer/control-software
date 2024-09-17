use proc_macro::TokenStream;
use quote::quote;
use syn::{
    bracketed,
    parse::Parse,
    parse_macro_input,
    token::Struct,
    DeriveInput,
    Ident,
    LitStr,
    Token,
};

fn _hash(data: &str) -> u32 {
    let mut prev_sum: u32 = 0;
    data.as_bytes()
        .iter()
        .map(|num| {
            prev_sum = prev_sum ^ *num as u32;
            prev_sum
        })
        .sum::<u32>()
}

#[proc_macro]
pub fn hash(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();

    let hash = _hash(&input);

    println!("Hash {hash}");

    quote! {
        #hash
    }
    .into()
}

#[proc_macro]
pub fn string_to_bytes(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();
    let bytes = input.as_bytes();

    let len = (bytes.len() >> 1).to_le_bytes();

    quote! {
        [#(#len,)*#(#bytes,)*]
    }
    .into()
}

struct IdentArray(Vec<Ident>);

impl Parse for IdentArray {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let content;
        bracketed!(content in input);
        let mut idents = Vec::new();
        while !content.is_empty() {
            idents.push(content.parse()?);
            let _: Token![,] = content.parse()?;
        }
        Ok(Self(idents))
    }
}

#[proc_macro]
pub fn numel(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as IdentArray).0.len();
    quote! {
        #input
    }
    .into()
}

static mut INDEX_COUNTER: u16 = u16::MAX;

#[proc_macro_derive(Serializer)]
pub fn serialize(item: TokenStream) -> TokenStream {
    let item = parse_macro_input!(item as DeriveInput);
    let mut name = item.ident.clone();
    let mut field_names: Vec<Ident> = Vec::new();
    let mut field_types = Vec::new();

    let generics = item.generics.clone();
    let mut generic_names: Vec<Ident> = item
        .generics
        .lifetimes()
        .into_iter()
        .map(|el| el.lifetime.ident.clone())
        .collect();

    generic_names.extend(
        item.generics
            .type_params()
            .into_iter()
            .map(|el| el.ident.clone()),
    );
    generic_names.extend(
        item.generics
            .const_params()
            .into_iter()
            .map(|el| el.ident.clone()),
    );

    match &item.data {
        syn::Data::Struct(s) => {
            field_names.extend(
                s.fields
                    .clone()
                    .iter()
                    .map(|el| &el.ident)
                    .filter(|el| el.is_some())
                    .map(|el| unsafe { el.clone().unwrap_unchecked() }),
            );
            field_types.extend(
                s.fields
                    .clone()
                    .iter()
                    .map(|el| el.ty.clone())
            );
            
        }
        t => {
            println!("FOUND TYPE {t:?}");
            panic!("Incompatible type.");
        }
    }

    let token = _hash(name.to_string().as_str());;
    let intermediate: Vec<proc_macro2::TokenStream> = field_names
        .iter()
        .map(|el| {
            quote! {
                let (n,intermediate_data) = self.#el.into_bytes();
                for el in 0..(n) {
                    assert!(ptr < Self::BUFFER_SIZE);
                    data[ptr] = intermediate_data[el];
                    ptr += 1;
                }
            }
        })
        .collect();

    let intermediate_decode: Vec<proc_macro2::TokenStream> = field_names
        .iter().zip(field_types.iter())
        .map(|(el,ty)| {
            quote! {
                let (n,intermediate_data) =match  #ty::from_bytes(ptr,data){
                    Ok(inner) => inner,
                    Err(_) => return Err(())
                };
                let #el = intermediate_data;
                ptr += n;
            }
        })
        .collect();

    let impl_line = match generic_names.len() {
        0 => quote!{#name},
        _ => quote!{#name <#(#generic_names,)*>},
    };

    // #item
    quote! {
        
        impl #generics #impl_line {
            const fn __size() -> usize {
                let mut size = core::mem::size_of::<#name>() + 4;
                
                const fn size_of<T: Serializable>() -> usize {
                    T::BUFFER_SIZE
                }
                #(
                    size += size_of::<#field_types>();
                )*
                size
    
            }

        }
        impl #generics Serializable for #impl_line {
                const BUFFER_SIZE: usize = {Self::__size()};
                type Error = ();
                /// Returns number of bytes used and buffer.
                fn into_bytes<'a>(&'a self) -> (usize, [u8; Self::BUFFER_SIZE]) {
                    let mut data = [0;Self::BUFFER_SIZE];
                    let token = #token.to_le_bytes();
                    data[0] = token[0];
                    data[1] = token[1];
                    data[2] = token[2];
                    data[3] = token[3];
                    let mut ptr = 4;
                    #(#intermediate)*
                    (ptr,data)
                }
                fn from_bytes<'a>(ptr: usize, data: &'a mut [u8]) -> Result<(usize, Self), Self::Error>
                where
                    Self: Sized {
                    let old_ptr = ptr.clone();
                    if data.len() < (ptr +4) {
                        return Err(());
                    }
                    let data_repr = [data[ptr],data[ptr + 1],data[ptr+2],data[ptr + 3]];
                    if u32::from_le_bytes(data_repr) != #token {
                        return Err(());
                    }

                    let mut ptr = ptr + 2;
                    #(
                        #intermediate_decode
                    )*
                    let ptr = ptr-old_ptr;

                    Ok((ptr,Self {
                        #(
                            #field_names,
                        )*
                    }))
                }
        }
    }
    .into()
}
